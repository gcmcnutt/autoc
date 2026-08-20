/* src_libcamera.cc — T021: the live frame source, as an APPLICATION of libcamera rather than a pipe (R8).
 *
 * The three things a pipe cannot give us, and where each lands here:
 *   1. per-frame metadata      -> request metadata: SensorTimestamp / ExposureTime / AnalogueGain
 *   2. request-level control   -> per-start ControlList: manual exposure+gain, FrameDurationLimits
 *   3. DMA buffers, zero-copy  -> FrameBufferAllocator + mmap once at open; FrameView borrows the plane
 *
 * Threading: libcamera delivers requestCompleted on its own event loop thread. Completed requests go
 * into a small mutex+condvar queue; next() pops one, exposes its plane as the FrameView, and requeues
 * the PREVIOUS request (the view stays valid until the caller asks for the next frame — exactly the
 * FrameView borrow contract).
 */
#include "src_libcamera.h"

#include <libcamera/libcamera.h>
#if defined(__aarch64__)
#include <arm_neon.h>
#endif

#include <condition_variable>
#include <cstring>
#include <deque>
#include <map>
#include <mutex>
#include <cstdlib>
#include <sys/mman.h>
#include <unistd.h>

using namespace libcamera;

namespace {

struct LcCtx {
    std::unique_ptr<CameraManager>       mgr;
    std::shared_ptr<Camera>              cam;
    std::unique_ptr<CameraConfiguration> config;
    std::unique_ptr<FrameBufferAllocator> alloc;
    Stream                              *stream = nullptr;
    std::vector<std::unique_ptr<Request>> requests;
    std::map<const FrameBuffer *, const uint8_t *> maps;   /* mmap'd once at open      */
    size_t   map_len = 0;
    uint32_t stride = 0, w = 0, h = 0;

    std::mutex               mtx;
    std::condition_variable  cv;
    std::deque<Request *>    done;
    Request                 *in_hand = nullptr;   /* the request whose plane the caller is looking at */
    bool                     stopping = false;

    void completed(Request *req) {
        if (req->status() == Request::RequestCancelled) return;
        {
            std::lock_guard<std::mutex> lk(mtx);
            done.push_back(req);
        }
        cv.notify_one();
    }
};

FrameStatus lc_next(FrameSource *self, FrameView *out)
{
    LcCtx *c = static_cast<LcCtx *>(self->ctx);

    /* Requeue the frame the caller just finished with — this is what keeps the pipeline fed. */
    if (c->in_hand) {
        c->in_hand->reuse(Request::ReuseBuffers);
        c->cam->queueRequest(c->in_hand);
        c->in_hand = nullptr;
    }

    Request *req;
    {
        std::unique_lock<std::mutex> lk(c->mtx);
        if (!c->cv.wait_for(lk, std::chrono::seconds(2), [c] { return !c->done.empty() || c->stopping; })) {
            fprintf(stderr, "libcamera: no frame for 2 s — pipeline stalled\n");
            return FRAME_ERROR;
        }
        if (c->stopping && c->done.empty()) return FRAME_END;
        req = c->done.front();
        c->done.pop_front();
    }
    c->in_hand = req;

    FrameBuffer *buf = req->buffers().at(c->stream);
    const uint8_t *data = c->maps.at(buf);

    std::memset(out, 0, sizeof *out);
    out->data   = data;                /* R8: the image. YUV420: plane 0 IS the u8 Y image.          */
    out->stride = static_cast<uint16_t>(c->stride);
    out->w      = static_cast<uint16_t>(c->w);
    out->h      = static_cast<uint16_t>(c->h);
    /* seq from the BUFFER metadata: it is the sensor's frame counter, so a dropped frame shows as a gap
     * here — which FrameView defines as an error to be seen, not smoothed over. */
    out->seq = buf->metadata().sequence;

    const ControlList &md = req->metadata();
    if (auto ts = md.get(controls::SensorTimestamp))
        out->t_us = static_cast<uint64_t>(*ts) / 1000u;   /* ns -> us, monotonic sensor clock */
    if (auto exp = md.get(controls::ExposureTime))
        out->exposure_us = static_cast<uint32_t>(*exp);
    if (auto gain = md.get(controls::AnalogueGain))
        out->gain_q8 = static_cast<uint16_t>(*gain * 256.0f + 0.5f);
    return FRAME_OK;
}

void lc_close(FrameSource *self)
{
    LcCtx *c = static_cast<LcCtx *>(self->ctx);
    {
        std::lock_guard<std::mutex> lk(c->mtx);
        c->stopping = true;
    }
    c->cam->stop();
    for (auto &m : c->maps)
        munmap(const_cast<uint8_t *>(m.second), c->map_len);
    c->alloc.reset();
    c->cam->release();
    c->cam.reset();
    c->mgr->stop();
    delete c;
    free(self);
}

} // namespace

extern "C" int bcn_libcamera_open(FrameSource **out, const BcnConfig *cfg, char *err, size_t err_len)
{
    unsigned w = 0, h = 0;
    if (sscanf(cfg->camera_mode, "%ux%u", &w, &h) != 2) {
        snprintf(err, err_len, "libcamera: [camera] mode \"%s\" is not WxH", cfg->camera_mode);
        return -1;
    }

    auto *c = new LcCtx();
    c->mgr = std::make_unique<CameraManager>();
    if (c->mgr->start() != 0) {
        snprintf(err, err_len, "libcamera: CameraManager failed to start");
        delete c; return -1;
    }
    {
        auto cams = c->mgr->cameras();
        if (!cams.empty()) c->cam = cams[0];
        /* cams' shared_ptrs MUST NOT outlive an error path's CameraManager: letting the vector
         * destruct after `delete c` was a real segfault (deleteLater onto a destroyed manager
         * thread — found by backtrace on the pisp bring-up, 2026-08-20). */
    }
    if (!c->cam) {
        snprintf(err, err_len, "libcamera: no cameras found (is the OV9281 ribbon seated?)");
        c->mgr->stop(); delete c; return -1;
    }
    if (c->cam->acquire() != 0) {
        snprintf(err, err_len, "libcamera: camera busy — another rpicam/beacon process holds it "
                               "(pkill rpicam-raw and retry)");
        c->cam.reset(); c->mgr->stop(); delete c; return -1;
    }

    /* THE ISP PATH, deliberately (Pi 5 pisp bring-up, 2026-08-20). The raw/CFE path on pisp only
     * emits 16-bit containers AND starved after its 2 internal image buffers in every configuration
     * tried (one-frame wedge; debug-log archaeology in the bench journal). The ISP's processed output
     * is the path this pipeline demonstrably services at speed, and it can emit R8 greyscale directly
     * — hardware 16->8, zero CPU. On vc4 the raw path was the free choice; here the ISP is. The
     * contract is FrameView u8 frames, not sensor-raw bytes: photometric caveat (ISP tuning curve sits
     * between sensor and samples) is noted for spec §5 photometry work, and denoise is forced OFF so
     * the point source stays a point.
     * The 8-bit SENSOR mode is still forced via sensorConfig (else the pipeline picks Y10 and caps at
     * 247.8 fps), and the frame duration is clamped to the mode's real floor (commanding below it
     * wedges the sensor after one frame — measured). */
    c->config = c->cam->generateConfiguration({ StreamRole::Viewfinder });
    if (!c->config || c->config->empty()) {
        snprintf(err, err_len, "libcamera: no Viewfinder stream role on this camera");
        c->cam->release(); c->cam.reset(); c->mgr->stop(); delete c; return -1;
    }
    StreamConfiguration &sc = c->config->at(0);
    sc.size = Size(w, h);
    /* YUV420, not R8: the mono ISP adjusts R8 to R16 (measured), but YUV420 always flows and its
     * plane 0 is exactly the u8 Y image core wants — same bytes, ISP does the depth conversion. */
    sc.pixelFormat = formats::YUV420;
    sc.bufferCount = 8;   /* pisp preps only 12 internal config/stats buffers; 16 requests starved the
                           * pipeline into Idle after one completion (2026-08-20). 8 = 26 ms of slack at
                           * 309 fps, and the A76+ISP path has no per-frame copy deficit to absorb. */
    if (!getenv("BCN_NO_SENSCFG")) {   /* bisection flag, pisp bring-up */
        SensorConfiguration sens;
        sens.outputSize = Size(w, h);
        sens.bitDepth = 8;
        c->config->sensorConfig = sens;
    }
    if (c->config->validate() == CameraConfiguration::Invalid) {
        snprintf(err, err_len, "libcamera: %ux%u R8 viewfinder rejected outright", w, h);
        c->cam->release(); c->cam.reset(); c->mgr->stop(); delete c; return -1;
    }
    /* R8 preferred; YUV420 acceptable (plane 0 IS the u8 image, stride-aware). Anything else, or a
     * geometry change, is refused — no silent substitution. */
    if (sc.size.width != w || sc.size.height != h ||
        (sc.pixelFormat != formats::R8 && sc.pixelFormat != formats::YUV420)) {
        snprintf(err, err_len, "libcamera: requested %ux%u R8, pipeline adjusted to %s %s — refusing "
                               "silent substitution; fix [camera] mode",
                 w, h, sc.size.toString().c_str(), sc.pixelFormat.toString().c_str());
        c->cam->release(); c->cam.reset(); c->mgr->stop(); delete c; return -1;
    }
    if (c->cam->configure(c->config.get()) != 0) {
        snprintf(err, err_len, "libcamera: configure() failed for %ux%u R8", w, h);
        c->cam->release(); c->cam.reset(); c->mgr->stop(); delete c; return -1;
    }
    c->stream = sc.stream();
    c->stride = sc.stride;
    c->w = w; c->h = h;

    c->alloc = std::make_unique<FrameBufferAllocator>(c->cam);
    if (c->alloc->allocate(c->stream) <= 0) {
        snprintf(err, err_len, "libcamera: buffer allocation failed");
        c->cam->release(); c->cam.reset(); c->mgr->stop(); delete c; return -1;
    }
    for (const auto &buf : c->alloc->buffers(c->stream)) {
        /* Map once, for the recorder's lifetime — never per frame (R6). Single-plane R8. */
        const FrameBuffer::Plane &pl = buf->planes()[0];
        c->map_len = pl.length;
        void *mem = mmap(nullptr, pl.length, PROT_READ, MAP_SHARED, pl.fd.get(), 0);
        if (mem == MAP_FAILED) {
            snprintf(err, err_len, "libcamera: mmap of a DMA buffer failed: %s", strerror(errno));
            c->cam->release(); c->cam.reset(); c->mgr->stop(); delete c; return -1;
        }
        c->maps[buf.get()] = static_cast<const uint8_t *>(mem);

        auto req = c->cam->createRequest();
        if (!req || req->addBuffer(c->stream, buf.get()) != 0) {
            snprintf(err, err_len, "libcamera: request setup failed");
            c->cam->release(); c->cam.reset(); c->mgr->stop(); delete c; return -1;
        }
        c->requests.push_back(std::move(req));
    }

    c->cam->requestCompleted.connect(c, &LcCtx::completed);

    /* Manual everything: auto-exposure/auto-gain WILL sabotage code capture — settling ramps read as
     * "signal" (README §Measured facts, learned twice). The AGC (spec §4) drives these DELIBERATELY,
     * per-request, when it lands; free-running auto is never acceptable. */
    ControlList ctl(c->cam->controls());
    if (!getenv("BCN_NO_CTRL")) {      /* bisection flag */
        ctl.set(controls::AeEnable, false);
        ctl.set(controls::ExposureTime, static_cast<int32_t>(cfg->exposure_min_us));
        ctl.set(controls::AnalogueGain, static_cast<float>(cfg->gain_min_q8) / 256.0f);
    }
    if (!getenv("BCN_NO_FDL")) {
        /* Clamp to the MODE's real minimum: commanding a frame duration below it does not saturate
         * gracefully — the sensor wedges after one frame (measured). The camera's control info knows
         * the floor once configure() has run. */
        int64_t us = 1000000 / cfg->fps;
        const auto it = c->cam->controls().find(&controls::FrameDurationLimits);
        if (it != c->cam->controls().end()) {
            const int64_t floor_us = it->second.min().get<int64_t>();
            if (us < floor_us) us = floor_us;
        }
        std::array<int64_t, 2> lim{ us, us };
        ctl.set(controls::FrameDurationLimits, Span<const int64_t, 2>(lim));
    }

    if (c->cam->start(&ctl) != 0) {
        snprintf(err, err_len, "libcamera: start() failed");
        c->cam->release(); c->cam.reset(); c->mgr->stop(); delete c; return -1;
    }
    for (auto &req : c->requests)
        c->cam->queueRequest(req.get());

    auto *src = static_cast<FrameSource *>(calloc(1, sizeof(FrameSource)));
    src->ctx = c;
    src->next = lc_next;
    src->close = lc_close;
    src->nominal_fps = cfg->fps;
    *out = src;
    return 0;
}
