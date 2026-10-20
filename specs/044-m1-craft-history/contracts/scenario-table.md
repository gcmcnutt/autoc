# Contract — startup scenario table (training / eval log)

Existing columns unchanged in meaning; appended columns:

| column | meaning | units |
|---|---|---|
| `clP` | roll damping draw (absolute) | — |
| `windMps` | wind speed draw at full strength | m/s |
| `turbX` | turbulence intensity multiplier draw | × |
| `gustLx` | Dryden length-scale multiplier draw | × |
| `thermX` | thermal strength multiplier draw | × |

Values are the full-strength draws; the flown values are `base + scale·(draw − base)` at the generation's variation scale.
