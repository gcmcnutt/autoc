# Contract — configuration (`inih`)

**Constitution VII binds here.** There are **no in-code defaults** for any key below. A missing key is a
startup error naming the key and exiting `1`. This is the 032 `cepGateThreshold` failure mode, and every
key here is exactly the kind that produced it: correct-looking results from a stale hardcoded value.

| section | keys |
|---|---|
| `[camera]` | `mode`, `fps`, `exposure_min_us`, `exposure_max_us`, `gain_min_q8`, `gain_max_q8` |
| `[code]` | `n_chips`, `chip_hz_nominal`, `chip_hz_candidates`, `code_a`, `code_b` |
| `[bank]` | `max_slots`, `scale_extents`, `alpha`, `beta`, `q_lock`, `q_drop`, `lock_health_lock`, `lock_health_drop`, `hold_max_age_ms`, `hold_max_cep_px` |
| `[agc]` | `exposure_target_lo`, `exposure_target_hi`, `integration_min_chips`, `integration_max_chips`, `roi_driven` (must be true in flight — spec §4/§9) |
| `[record]` | `mode`, `path`, `ring_seconds`, `burst_frames`, `burst_every`, `trigger` |
| `[sched]` | `acquire_cost_us_per_pass`, `acquire_passes_max` — the cost model R3's replay virtualisation replays against |
| `[sync]` | `fiducial_enabled`, `fiducial_period_s`, `msp_uart`, `msp_baud` (spec §7.1.2) |

**`hold_max_age_ms` and `hold_max_cep_px` MUST equal the §3.1 validity bounds.** The state machine and the
scoring metric read the same two numbers from the same place, so they cannot drift apart.

`config_hash` is computed over the fully-resolved config and stamped into every record and every recording
(spec §16.2) — so any artifact can be traced to the settings that produced it.
