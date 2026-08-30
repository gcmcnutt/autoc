/* emit_json.c — T060. One JSON line per record; a PROJECTION generated from the same struct the binary
 * sink encodes, never alongside it. Colour convention preserved: code A = PORT (red), B = STARBOARD
 * (green) — beacon_display.py and every plot rely on it. */
#include "emit_json.h"
#include <stdio.h>
#include <string.h>
#include <unistd.h>

int bcn_emit_json(int fd, const BcnRecord *r)
{
    char buf[4096];
    int n = snprintf(buf, sizeof buf,
        "{\"t_us\":%llu,\"seq\":%u,\"tick\":%u,\"n\":%u,\"slots\":%u,\"deadline_us\":%d,\"tracks\":[",
        (unsigned long long)r->t_us, r->seq, r->tick_index, r->n_tracks, r->n_slots_used,
        r->deadline_margin_us);
    uint8_t k;
    for (k = 0; k < r->n_tracks && k < BCN_MAX_TRACKS; k++) {
        const BcnTrack *t = &r->tracks[k];
        n += snprintf(buf + n, sizeof buf - (size_t)n,
            "%s{\"code\":\"%s\",\"x\":%.2f,\"y\":%.2f,\"vx\":%.1f,\"vy\":%.1f,"
            "\"xp\":%.2f,\"yp\":%.2f,\"cep\":%.2f,\"q\":%.2f,\"lh\":%.2f,"
            "\"chip_hz\":%.2f,\"phase\":%u,\"scale\":%u,\"tint\":%u,\"age_ms\":%u,\"flags\":%u}",
            k ? "," : "",
            t->code_id ? "B" : "A",                     /* B = STARBOARD/green, A = PORT/red */
            t->x / 256.0, t->y / 256.0, t->vx / 256.0, t->vy / 256.0,
            t->x_pred / 256.0, t->y_pred / 256.0, t->cep / 256.0,
            t->q / 256.0, t->lock_health / 256.0,
            t->chip_hz / 256.0, t->chip_phase, t->scale, t->t_int_chips, t->age_ms, t->flags);
        if ((size_t)n >= sizeof buf - 200) break;
    }
    n += snprintf(buf + n, sizeof buf - (size_t)n, "]}\n");
    return write(fd, buf, (size_t)n) == (ssize_t)n ? 0 : -1;
}
