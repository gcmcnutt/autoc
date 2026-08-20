/* bcnr_info.c — inspect a .bcnr recording: header, frame census, gap structure, timing sanity.
 * The T025 verification tool, and the first thing to run on any capture that looks wrong. */
#include "container.h"
#include "src_replay.h"
#include "frame.h"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

int main(int argc, char **argv)
{
    char err[256];
    FrameSource *src = NULL;
    const BcnContainerHeader *h;
    FrameView fv;
    FrameStatus st;
    uint64_t n = 0, gaps = 0, bursts = 0, dumps = 0;
    uint64_t first_t = 0, last_t = 0, min_dt = ~0ull, max_dt = 0, sum_dt = 0;
    uint32_t prev_seq = 0, burst_len = 0, min_burst = ~0u, max_burst = 0;
    uint32_t exp_lo = ~0u, exp_hi = 0;
    uint16_t gain_lo = 0xFFFF, gain_hi = 0;
    int have_prev = 0;

    if (argc != 2) { fprintf(stderr, "usage: bcnr_info <clip.bcnr>\n"); return 1; }
    if (bcn_replay_open(&src, argv[1], err, sizeof err) != 0) {
        fprintf(stderr, "bcnr_info: %s\n", err);
        return 2;                              /* version mismatch names both versions — Constitution V */
    }
    h = bcn_replay_header(src);
    printf("%s\n", argv[1]);
    printf("  container v%u  %ux%u @%u fps nominal  mode %s\n",
           h->format_version, h->width, h->height, h->nominal_fps,
           h->mode == BCN_MODE_CONTINUOUS ? "continuous" :
           h->mode == BCN_MODE_RING ? "ring" : "burst");
    printf("  build_id %016" PRIx64 "  config_hash %016" PRIx64 "  start_t_us %" PRIu64 "\n",
           h->build_id, h->config_hash, h->start_t_us);

    while ((st = src->next(src, &fv)) == FRAME_OK) {
        const BcnFrameHeader *fh = bcn_replay_last_frame_header(src);
        if (!have_prev) { first_t = fv.t_us; }
        else {
            if (bcn_seq_is_gap(prev_seq, fv.seq)) { gaps++; if (burst_len) { if (burst_len < min_burst) min_burst = burst_len; if (burst_len > max_burst) max_burst = burst_len; } burst_len = 0; }
            else {
                uint64_t dt = fv.t_us - last_t;
                if (dt < min_dt) min_dt = dt;
                if (dt > max_dt) max_dt = dt;
                sum_dt += dt;
            }
        }
        burst_len++;
        if (fh->flags & BCN_FR_BURST_START) bursts++;
        if (fh->flags & BCN_FR_TRIGGER_DUMP) dumps++;
        if (fh->inav_t_us != 0u)
            printf("  frame %u carries inav_t_us %" PRIu64 "\n", fv.seq, fh->inav_t_us);
        if (fv.exposure_us < exp_lo) exp_lo = fv.exposure_us;
        if (fv.exposure_us > exp_hi) exp_hi = fv.exposure_us;
        if (fv.gain_q8 < gain_lo) gain_lo = fv.gain_q8;
        if (fv.gain_q8 > gain_hi) gain_hi = fv.gain_q8;
        prev_seq = fv.seq; last_t = fv.t_us; have_prev = 1;
        n++;
    }
    if (burst_len) { if (burst_len < min_burst) min_burst = burst_len; if (burst_len > max_burst) max_burst = burst_len; }
    if (st == FRAME_ERROR) { fprintf(stderr, "bcnr_info: stream ended in ERROR (truncated?)\n"); src->close(src); return 2; }

    printf("  frames %" PRIu64 "  seq gaps %" PRIu64 "  burst starts %" PRIu64 "  trigger dumps %" PRIu64 "\n",
           n, gaps, bursts, dumps);
    if (n > 1) {
        uint64_t contig = n - 1 - gaps;
        printf("  span %.3f s  intra-burst dt min/mean/max %.0f/%.0f/%.0f us (%.2f fps)\n",
               (double)(last_t - first_t) / 1e6,
               (double)min_dt, contig ? (double)sum_dt / (double)contig : 0.0, (double)max_dt,
               contig ? 1e6 * (double)contig / (double)sum_dt : 0.0);
        printf("  burst length min/max %u/%u frames\n", min_burst, max_burst);
    }
    printf("  exposure %u..%u us  gain %.2f..%.2f\n", exp_lo, exp_hi, gain_lo / 256.0, gain_hi / 256.0);
    src->close(src);
    return 0;
}
