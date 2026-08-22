# Phase 5 handoff — INAV accel over MSP, for a fresh context

> ⚠️ **STATUS 2026-08-22 (end of bench session): P5-1, P5-2 and P5-3 are DONE.** This document describes
> the state BEFORE that work and is kept for its environment notes and the three convention traps, which
> remain accurate. For what is actually left, read [`tasks.md`](tasks.md) § Phase 5 — specifically the
> TOOLCHAIN + FORMAT STATE block (flight log v4, the nn2cpp CLI change, the renderer unit fix) and P5-4 /
> P5-5 / P5-6. ⛔ Do NOT use §7's "P5-3 is the parallel path" or §8's "what done looks like" as current
> scope: both are satisfied. The accel convention is measured, not predicted — bench FRD `[0,0,−1]` /
> `[+1,0,0]` / `[0,−1,0]` confirmed by three independent parsers and cross-checked against INAV's own
> blackbox to **0.34 milli-g**.

**Written 2026-08-22 from the 041-t7 bake context.** You are picking up **Phase 5** of 041 while a training
run continues on this machine. This file is orientation and boundaries only — it does **not** restate the
spec.

⛔ **[`tasks.md`](tasks.md) § Phase 5 GOVERNS.** P5-1 carries the complete T072 spec: which field to source,
the wire encoding, the frame convention, the target order. It is finished work — **do not re-derive it**,
and do not treat anything here as overriding it.

---

## 1. Why you are in a fresh context

Phase 5 is a different repo (`~/inav`), a different toolchain, and a different domain (STM32 firmware, MSP
wire protocol, bench hardware) from the autoc analytics work that filled the originating context. There is
no useful carry-over. Everything you need is in-repo.

## 2. Environment — VERIFIED 2026-08-22, do not re-check

| thing | state |
|---|---|
| `~/inav` | present, branch `autoc`, **clean tree** |
| `~/inav` HEAD | **`63cffaf4f`** — which *is* the commit P5-1 tells you to copy the shape of |
| the MSP case to extend | `src/main/fc/fc_msp.c:672`, `case MSP2_INAV_LOCAL_STATE:` |
| the field to source | `src/main/sensors/acceleration.c:568` — `acc.accADCf[axis] = (float)accADC[axis] / acc.dev.acc_1G` |
| size of the template change | `63cffaf4f` was **8 lines in one file** |

⭐ HEAD already being the reference commit is convenient: `git show 63cffaf4f` is both "the example" and
"the diff that produced the current state".

## 3. ⛔ THE BOUNDARY THAT MATTERS — a bake is running

**`autoc` training run t7 is live** (gen ~580 of 800, pid in `logs/autoc-041-t7-m1-inputscale.log`).

* ⛔ **Do NOT build autoc in the main worktree.** Workers re-exec `build/autoc`; overwriting it kills a
  multi-day run. A reflex `./rebuild.sh` or `cmake --build build` is the failure mode.
* ✅ INAV work touches none of this — different repo, different build tree. Just never let a "let me just
  rebuild and check" reflex land in `/home/gmcnutt/autoc`.
* If you genuinely need an autoc binary, use a **separate `git worktree`** (it gets its own `build/`).

## 4. Three traps, all previously resolved — confirm, don't re-derive

1. ⚠️ **The bench table in `docs/COORDINATE_CONVENTIONS.md` is in INAV FLU COUNTS** (`acc_1G ≈ 2048`).
   Divide by 2048 **and** flip y/z before comparing to anything FRD. An earlier "flip it" instruction was
   **withdrawn** after being resolved against `~/inav @ 63cffaf4`. Getting this backwards is invisible in
   sim and wrong in the air.
2. ⚠️ **Source `acc.accADCf`, not `accADC` and not `acc.dev.ADCRaw`.** Only `accADCf` is
   alignment-corrected and already in g. The others bake each board's own misalignment in differently —
   bench roll is −16° vs flight, so it would look fine on the table and be wrong airborne.
3. ⚠️ **The wire carries INAV-native FLU, UNFLIPPED**, exactly as quat and gyro already do. The FLU→FRD flip
   belongs at the msplink boundary, once, beside the other two. Settled 2026-08-11.

## 5. Read in this order

1. [`tasks.md`](tasks.md) § Phase 5 — the spec (P5-1 … P5-4)
2. `docs/COORDINATE_CONVENTIONS.md` — the bench table, and trap 1
3. `specs/031-beacon-camera/bench-journal.md` — **CLAUDE.md requires this before any bench work**
4. `docs/inav-signal-path-audit.md` — prior INAV signal-path analysis

## 6. ⚠️ A DOCUMENTATION GAP — likely your first task

`docs/toolchains.md` has **no INAV row**. It covers autoc, Python, xiao, the AVR emitter, Lattice Diamond,
KiCad and iverilog — but not INAV, despite CLAUDE.md pointing there for "how each toolchain is
built/driven". The build and flash procedure for `MAMBAF722_2022A` / `MATEKF722MINI` is not written down
anywhere in this repo.

⛔ **Ask the operator for the actual invocation rather than guessing** — a wrong flash procedure on a flight
controller is expensive. Then add the row:

```markdown
| **INAV FC firmware** (021/041) | <toolchain> | <where it runs> | <build cmd>; targets `MAMBAF722_2022A` (bench) / `MATEKF722MINI` (flight); ⚠️ disconnect GPS before flashing |
```

⭐ Do this **before** P5-1, not after. It is the reason this phase is called long-lead — operator
2026-08-18: *"this is required ahead of all given the complexity of setting up the programming."*

## 7. Scope note — P5-3 needs no hardware

Three of the four channels the xiao currently zeroes (`SPECIFIC_ENERGY`, `BOUNDARY_CLOSURE_RATE`,
`SCORE_GRAD_*`) derive from data already on board and can be written **without waiting on P5-1**. Only
`ACCEL_*` needs the MSP extension. If bench access is blocked, P5-3 is the parallel path.

⚠️ P5-3 has its own catch, stated in the task: the cone constants are **not on the xiao today** and must be
baked in alongside the arena template, or the gradient cannot be computed there.

## 8. What "done" looks like

P5-2 is the gate, and it is deliberately checkable on a table rather than inferred from a flight. In **FRD,
after the msplink flip**, three static attitudes:

| attitude | expected |
|---|---|
| level | `[0, 0, −1]` |
| nose up | `[+1, 0, 0]` |
| right wing down | `[0, −1, 0]` |

⛔ **Bench target first** (`MAMBAF722_2022A`), then flight (`MATEKF722MINI`). **Disconnect the GPS before
flashing.**

---

## 9. Context you may be asked about — the bake

Not your task, but you may be asked. **041-t7 is the new M1 baseline of record** (see
[`baseline.md`](baseline.md)) — best −73,899 at gen 551, beating the all-time M1 record of −55,270 while
*lowering* airframe load (6 g exceedance 4.2× lower than the prior M1). The cause was **P2-8 input
scaling**, not the objective work that preceded it.

⛔ **Relevant to you**: pre-P2-8 genomes load cleanly and fly **wrong** — the input *scales* changed while
the layout did not (still `float[45]`). **P2-9** exists to make `nn2cpp` bake a scale signature so this
fails loudly instead of silently. If flight work starts before P2-9 lands, that is the hazard to raise.
