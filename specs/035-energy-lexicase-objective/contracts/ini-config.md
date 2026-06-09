# Contract — ini config changes (FR-P07/P08, FR-003, FR-P13)

All keys flow through the `AUTOC_CONFIG_FIELDS(X)` X-macro (034 US3): one edit to the macro list
updates decl + parse + startup-print. No hand-written print lines.

## New / changed keys

| Key | Type | Default | Files | FR |
|---|---|---|---|---|
| `S3Bucket` | string | per-mode | all inis | FR-P07/P08 |
| `LexicaseEpsilonMode` | string | `mad` | training inis (autoc.ini, autoc-tracker.ini, autoc-basic-m1.ini) | FR-003 |
| `TrackerSourceRun` | string | — | autoc-eval-tracker.ini (repoint) | FR-P13 |

## Per-mode `S3Bucket` values (after admin bucket creation)

| ini | mode | S3Bucket |
|---|---|---|
| `autoc.ini` | pathgen (M1 prod) | `autoc-m1` |
| `autoc-basic-m1.ini` | pathgen (M1 smoke) | `autoc-m1` |
| `autoc-tracker.ini` | tracker (M2) | `autoc-m2` |
| `autoc-eval.ini` | eval | `autoc-eval` |
| `autoc-eval-tracker.ini` | eval | `autoc-eval` |
| `autoc-eval-visual.ini` | eval | `autoc-eval` |

(Until admin creates the buckets, these stay at `autoc-storage` / `autoc-storage-tracker` /
`autoc-eval-arm`; the cutover is a config-only flip per R6.)

## `LexicaseEpsilonMode`
```
LexicaseEpsilonMode = mad        # default — per-axis MAD-relative epsilon (FR-003/R2)
# LexicaseEpsilonMode = constant # historical: fixed 0.5 floor, bit-reproduces prior runs (SC-003)
```

## `TrackerSourceRun` repoint (FR-P13)
`autoc-eval-tracker.ini:42` currently → `autoc-9223370258388840205-2026-05-11…/gen9999.dmp`
(0 objects, dangling). Repoint to the live M2 source (the `autoc-tracker.ini` source, in
`autoc-m2` post-migration). Loader MUST fail loud if the key is absent (no silent substitute).

## Startup-print verification
The config-dump test (034 T028) asserts every active key prints; the two new keys are covered by
adding them to the macro list — no separate print code.
