# Contract — new / changed ini keys (044)

| key | file(s) | type | default | validation |
|---|---|---|---|---|
| `CraftClPSigma` | autoc.ini, autoc-eval.ini | double | 0.10 | ≥ 0; must be set (non-zero) when craft variations are on, like `CraftCmQSigma` |
| `CraftCmQSigma` | autoc.ini, autoc-eval.ini | double | 0.5 (was 0.6) | ≥ 0 |
| `EvalVariationScale` | autoc-eval.ini (eval mode) | double | −1 | −1 or 0…1, else startup error |

- Config contract test count rises by 2 (`CraftClPSigma`, `EvalVariationScale`).
- `autoc-eval.ini` mirrors `autoc.ini` for all variation keys (existing rule).
- String-valued keys keep comments on their own line (inline `#` is not stripped for strings).
