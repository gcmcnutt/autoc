# 041 P2-7 — Objective amendment: the energy axis goes back to throttle power

**2026-08-19, after t4 stopped at gen 511.** Operator: *"pretty sure we need to update objective — with
minimum toying with weights — this should be fundamental."*

⛔ **This amendment REVERSES P2-5 and corrects a premise in [spec.md](spec.md) § Why.** It is written up
rather than quietly applied because the premise is load-bearing: it is one of the three reasons 041 exists.

---

## 1. The premise that turned out to be wrong

`spec.md` states, as reason 2 of 3 for the whole feature:

> **The objective is missing its main term.** The policy cannot observe its own energy (no altitude input),
> so **every prior energy objective muted the whole regiment.**

⚠️ **The second half is contradicted by 035's own outcome document**, which is the primary record for the
only prior *lexicase* energy objective:

> **M1 verdict: ENERGY WORKS** (energy-lexicase, no tracking collapse) … The clearest energy signal is
> throttle amplitude falling **over the run**: `mag_throttle` 0.93 (gen 100) → **0.72** (gen 800). The
> controller learned to *spend less throttle*. Contrast the 034 energy-**scalar** run, which **froze
> throttle pinned at ~0.997** (a dead axis).
> — [035/outcome.md](../035-energy-lexicase-objective/outcome.md)

The correct history is not "energy objectives mute". It is **scalar aggregation mutes**:

| feature | shape | result |
|---|---|---|
| 033 US2 | **scalar** smoothness × tracking | ❌ stalled at gen 457, streak 1/11 of baseline |
| 034 | **scalar** energy | ❌ froze throttle at ~0.997, "a dead axis" |
| **035 US1** | **lexicase** throttle integral | ✅ **worked** — de-pegged throttle 0.93 → 0.72 |

That is [project_scalar_multiobjective_collapse](../../.claude/projects/-home-gmcnutt-autoc/memory/project_scalar_multiobjective_collapse.md)
exactly, and it is about the **aggregation rule**, not about energy. 041 generalised it one step too far and
replaced the term that had the record.

## 2. What t4 measured

t4 ran the P2-5 Es-destroyed axis to gen 511 and reproduced the **034** signature, not the 035 one:

| | 034 (scalar energy) | 035 (lexicase power) | **041-t4 (Es destroyed)** |
|---|---|---|---|
| throttle | pinned ~0.997 | 0.93 → **0.72** | **1.000 on 100% of 129,732 ticks** |
| pctInStreak | — | ~40% | **3.2%** |
| best | — | −37,690 | −14,855 |

For scale: prior M1 30.9%, 034 origm1 41.5%. t4 reached **3.2%**.

⚠️ **Confounded** — t4 also changed the input vector and the arena, so not all of the gap is the axis. But
the throttle peg points at the axis specifically, because throttle amplitude is the variable 035 identified
as its mechanism.

## 3. Why `Es` destroyed cannot serve as the cost term

⭐ **It charges for the RESULT, not the EXPENDITURE.** `energy_score = Σ max(0, −Ps)·dt` only accrues when
`Es` FALLS. Full throttle *raises* `Es`, so pinning the stick is not merely uncharged — it is **rewarded**.

The only remaining route to losing energy is drag, and drag is dominated by load: measured on t4 gen 395,
`corr(mean load, energy destroyed) = **+0.723**` per scenario. So the axis silently became **a penalty on
manoeuvring** — the opposite of what an energy objective is for, and directly opposed to tracking, which
requires manoeuvring.

It was not *muting* (measured `corr(destroyed, stepScore) = +0.028`, still orthogonal, so lexicase held).
It simply could not reach the behaviour it was named for.

## 4. The change

| | before (P2-5) | after (P2-7) |
|---|---|---|
| `energy_score` | `Σ max(0, −Ps)·dt` | **`Σ throttleEnergyStep(out_th)·kCadenceTickScale`** (035 FR-001b) |
| lexicase axes | `score`, `energy_score` | unchanged — **still two** |
| `SPECIFIC_ENERGY` input | slot 36 | **unchanged, kept** |
| `Es`/`Ps` in the dmp | recorded | **unchanged, kept** |
| `energy_progress.py` | plots Ps | **unchanged, kept as diagnostic** |
| `VariationRampStep` | 0 | **0 — stays off** (operator: M2 shows strong learning without it, repeatedly) |

⭐ **041's real contribution survives untouched.** The policy can now *observe* its own energy — TA03 showed
it never could — and that was always the more defensible half of the P2-2 work. What is withdrawn is only
`Es` as the *selection* term, which was never proven.

**New guard**: `ThrottlePowerAxis.FullThrottleCostsStrictlyMoreThanModulating_PEG_GUARD` asserts sustained
max power costs strictly more than an equal-mean modulated command, plus the convexity that makes it prefer
modulation. A linear cost is indifferent between steady half power and bang-bang, and an indifferent axis
can never de-peg the stick. That test would have failed on the P2-5 axis.

## 5. ⚠️ What this does NOT explain — open before the next bake

**041-t1 ran the OLD throttle axis and still reached only 16.1%**, against the prior M1's 30.9%. So
something besides the energy axis differs between 038-t5 and 041-t1, and restoring the axis does not
address it. ⚠️ Note the word is *differs*, not *regressed* — see item 1, which offers a reading under which
nothing is broken at all. Candidates, none yet tested:

1. ⭐ **the input-vector growth (37 → 45), operator's reading 2026-08-19**: *"the 16% could be because
   larger search space, recurrent layer filling up"* — i.e. NOT a defect but a **capacity/search cost**. A
   wider input layer enlarges the weight space the GA must search, and a recurrent layer takes longer to
   settle useful state, so a slower climb at equal generation count is the EXPECTED shape rather than a
   regression to hunt. ⚠️ This reading is testable and currently untested: it predicts t1 would have caught
   up given more generations, where a defect would predict a permanent ceiling. Do not treat it as settled;
2. `VariationRampStep` 0 — full variation from gen 1, new at 041 versus 034/035 (operator has decided this
   stays off, so it becomes a known difference rather than a suspect to remove);
3. the arena move (t4 only — t1 predates it).

⛔ **Do not read a good next bake as vindication of item 1 or 3** — the next run changes the axis only, so
it is a clean read on the axis and on nothing else.

## 6. Operator's four costs, and where each one lands

> *"high power costs, being low costs, maneuvering costs, aggressive costs"*

| cost | mechanism after this change | new term needed? |
|---|---|---|
| **high power** | the restored convex throttle integral | ❌ no — this IS the change |
| **being low** | `SPECIFIC_ENERGY` is an input; the deck is the datum, so low ⇒ low `Es` and the policy can see it | ❌ no |
| **manoeuvring** | induced drag bleeds airspeed ⇒ throttle must rise ⇒ the power axis charges it | ❌ no — physically routed, 035's argument |
| **aggressive** | ⚠️ **not currently priced.** `stability_score` exists and its axis is OFF (035 FR-008: "a control-amplitude whip … smoothness must EMERGE from energy via induced drag") | open |

**Recommendation: change one thing.** Restore the power axis, re-bake, and read it against 035's ~40%. If
aggressiveness is still wrong at that point, the stability axis is the next single change — and it will be
readable because nothing else moved. Two changes at once is what makes t4 hard to attribute today.
