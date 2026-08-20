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

#include <condition_variable>
#include <cstring>
#include <deque>
#include <map>
#include <mutex>
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
    out->data   = data;
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
    auto cams = c->mgr->cameras();
    if (cams.empty()) {
        snprintf(err, err_len, "libcamera: no cameras found (is the OV9281 ribbon seated?)");
        c->mgr->stop(); delete c; return -1;
    }
    c->cam = cams[0];
    if (c->cam->acquire() != 0) {
        snprintf(err, err_len, "libcamera: camera busy — another rpicam/beacon process holds it "
                               "(pkill rpicam-raw and retry)");
        c->mgr->stop(); delete c; return -1;
    }

    c->config = c->cam->generateConfiguration({ StreamRole::Raw });
    if (!c->config || c->config->empty()) {
        snprintf(err, err_len, "libcamera: no RAW stream role on this camera");
        c->cam->release(); c->mgr->stop(); delete c; return -1;
    }
    StreamConfiguration &sc = c->config->at(0);
    sc.size = Size(w, h);
    sc.pixelFormat = formats::R8;
    /* 16, not the customary 4-8: the dmabuf mmap is write-combine, and one uncached 256 KB read costs
     * ~4 ms on the A53 — slightly MORE than the 3.6 ms frame period, so a consumer accrues ~0.5 ms of
     * deficit per copied frame. 16 buffers = 58 ms of slack absorbs a full 80-frame burst's deficit
     * (~40 ms), which 8 buffers (29 ms) demonstrably did not (2026-08-19: drops clustered at burst
     * tails). Continuous full-raw on this host stays ~215 fps and that is a HOST limit, not a bug —
     * continuous is the Pi 5 flight mode (spec §8.2); the bench records bursts. */
    sc.bufferCount = 16;
    if (c->config->validate() == CameraConfiguration::Invalid) {
        snprintf(err, err_len, "libcamera: %ux%u R8 rejected outright", w, h);
        c->cam->release(); c->mgr->stop(); delete c; return -1;
    }
    /* validate() may ADJUST rather than reject — a silently different geometry would record a container
     * whose header lies. Refuse instead (Constitution VII's spirit: no silent substitution). */
    if (sc.size.width != w || sc.size.height != h || sc.pixelFormat != formats::R8) {
        snprintf(err, err_len, "libcamera: requested %ux%u R8, pipeline adjusted to %s %s — refusing "
                               "silent substitution; fix [camera] mode",
                 w, h, sc.size.toString().c_str(), sc.pixelFormat.toString().c_str());
        c->cam->release(); c->mgr->stop(); delete c; return -1;
    }
    if (c->cam->configure(c->config.get()) != 0) {
        snprintf(err, err_len, "libcamera: configure() failed for %ux%u R8", w, h);
        c->cam->release(); c->mgr->stop(); delete c; return -1;
    }
    c->stream = sc.stream();
    c->stride = sc.stride;
    c->w = w; c->h = h;

    c->alloc = std::make_unique<FrameBufferAllocator>(c->cam);
    if (c->alloc->allocate(c->stream) <= 0) {
        snprintf(err, err_len, "libcamera: buffer allocation failed");
        c->cam->release(); c->mgr->stop(); delete c; return -1;
    }
    for (const auto &buf : c->alloc->buffers(c->stream)) {
        /* Map once, for the recorder's lifetime — never per frame (R6). Single-plane R8. */
        const FrameBuffer::Plane &pl = buf->planes()[0];
        c->map_len = pl.length;
        void *mem = mmap(nullptr, pl.length, PROT_READ, MAP_SHARED, pl.fd.get(), 0);
        if (mem == MAP_FAILED) {
            snprintf(err, err_len, "libcamera: mmap of a DMA buffer failed: %s", strerror(errno));
            c->cam->release(); c->mgr->stop(); delete c; return -1;
        }
        c->maps[buf.get()] = static_cast<const uint8_t *>(mem);

        auto req = c->cam->createRequest();
        if (!req || req->addBuffer(c->stream, buf.get()) != 0) {
            snprintf(err, err_len, "libcamera: request setup failed");
            c->cam->release(); c->mgr->stop(); delete c; return -1;
        }
        c->requests.push_back(std::move(req));
    }

    c->cam->requestCompleted.connect(c, &LcCtx::completed);

    /* Manual everything: auto-exposure/auto-gain WILL sabotage code capture — settling ramps read as
     * "signal" (README §Measured facts, learned twice). The AGC (spec §4) drives these DELIBERATELY,
     * per-request, when it lands; free-running auto is never acceptable. */
    ControlList ctl(c->cam->controls());
    ctl.set(controls::AeEnable, false);
    ctl.set(controls::ExposureTime, static_cast<int32_t>(cfg->exposure_min_us));
    ctl.set(controls::AnalogueGain, static_cast<float>(cfg->gain_min_q8) / 256.0f);
    {
        const int64_t us = 1000000 / cfg->fps;
        std::array<int64_t, 2> lim{ us, us };
        ctl.set(controls::FrameDurationLimits, Span<const int64_t, 2>(lim));
    }

    if (c->cam->start(&ctl) != 0) {
        snprintf(err, err_len, "libcamera: start() failed");
        c->cam->release(); c->mgr->stop(); delete c; return -1;
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
