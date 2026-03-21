# Flight 2026-03-20 Timing Analysis
# Pipeline: INAV sensor → MSP → xiao → NN eval → cache → MSP send → INAV rcData → PID → servo

## Event Chain (per-span, times relative to xiao NN enable)

| Event | Span 1 | Span 2 | Span 3 | Span 4 | Span 5 | Mean | Jitter |
|-------|--------|--------|--------|--------|--------|------|--------|
| BOXMSPRCOVERRIDE on | -186ms | -252ms | -231ms | -222ms | -188ms | -216ms | ±28ms |
| **Xiao NN enable** | +0ms | +0ms | +0ms | +0ms | +0ms | +0ms | — |
| Freshness guard done | +781ms | +800ms | +786ms | +794ms | +794ms | +791ms | ±7ms |
| First INAV rcData Δ | +781ms | +800ms | +802ms | +794ms | +794ms | +794ms | ±8ms |

## Steady-State Pipeline Latency (after freshness guard)

| Metric | Span 1 | Span 2 | Span 3 | Span 4 | Span 5 | Mean | Jitter |
|--------|--------|--------|--------|--------|--------|------|--------|
| Nav→NN eval | 2.8±1.9 | 2.7±2.0 | 2.2±1.3 | 2.3±1.3 | 2.2±1.3 | **2.5ms** | [1,11]ms |
| NN interval | 101.0±9.2 | 100.3±8.0 | 100.5±10.1 | 100.7±9.3 | 100.7±10.0 | **100.7ms** | [78,125]ms |
| NN→INAV rcData | 45.2±32.9 | 48.3±39.4 | 42.7±37.3 | 44.1±30.6 | 58.8±31.2 | **47.8ms** | [0,107]ms |

## Clock Drift

| Span | Clock offset (inav_ms - xiao_ms) | Δ from Span 1 |
|------|----------------------------------|---------------|
| 1 | 586ms | +0ms |
| 2 | 550ms | -36ms |
| 3 | 530ms | -56ms |
| 4 | 482ms | -104ms |
| 5 | 441ms | -145ms |

Drift rate: **-1.04 ms/sec** (586ms → 441ms over 139s)

## Pipeline Summary

```
INAV sensor (1000Hz)
  │
  ├──[~0-50ms]── MSP poll by xiao (100ms cycle)
  │                   │
  │              NN eval (~3ms)
  │                   │
  │              cache RC commands
  │                   │
  ├──[0-50ms]─── MSP send ticker (50ms cycle, unsynchronized)
  │                   │
  │              INAV receives MSP_SET_RAW_RC
  │                   │
  │              mspOverrideChannels() applies to rcData
  │                   │
  │              expo → rates → PID → mixer (ACRO mode)
  │                   │
  └──────────── servo/motor output

Total: sensor → rcData = 48ms avg (NN eval + send cycle)
Freshness guard: 791ms (one-time at activation)
Clock drift: 1.0 ms/sec between xiao and INAV oscillators
```

## Action Items for Next Flight

1. **T221a** Remove MSP_ARM_CYCLE_COUNT=2 delay (unnecessary, INAV has 790ms guard)
2. **T221b** Use MANUAL mode not ACRO (sim applies commands directly, ACRO adds PID)
3. **T221c** Refactor to single 20Hz loop (eliminates 0-50ms send jitter)
4. **T222** Log rabbit world position in NN line
