# 042 — Phase 0 research

**No `NEEDS CLARIFICATION` markers survived into Technical Context.** The 2026-08-19 clarify session
(spec §Clarifications) closed the five spec-level unknowns. What follows are the **engineering** decisions
this plan makes, each with the alternative it rejects.

---

## R1 — Language split: C11 `core/`, C++17 `io/`+`app/`

**Decision**: `core/` is C11 with zero dependencies. C++17 appears only where libcamera's API requires it.

**Rationale**: the operator specified native C twice; more importantly the dependency boundary is what
buys replay parity, hardware-free CI, and the cheap WSL2 cross-compile (plan §Structure Decision). C11 makes
"no hidden allocation, no hidden dispatch" checkable by reading rather than by trusting. GoogleTest still
tests it through `extern "C"`.

**Alternatives**: (a) *all C++17, matching the rest of the repo* — rejected: it drags a toolchain
expectation into `core/` for no benefit, and the repo's C++17 convention is about the sim engine, not this.
(b) *all C11 including the shell* — rejected: libcamera has no stable C API; a hand-rolled C wrapper is
strictly more code to maintain than a thin C++ file.

---

## R2 — Fixed-point, not float, in the correlator hot paths

**Decision**: `int16` samples, `int32` accumulators, Q-format documented at each declaration. Any surviving
float is annotated with the reason.

**Rationale**: **bit-exact replay across A53, A76 and Grace** is a hard requirement, and float with FMA
contraction does not deliver it — the same source produces different results on different microarchitectures
unless contraction is pinned. Integer is also the NEON-friendly path (`vmlal_s16`) and the natural domain
for an 8-bit sensor. This is the spirit of Constitution VI carried into a C11 setting.

**Alternatives**: (a) *float with `-ffp-contract=off` and no `-ffast-math`* — workable but fragile; one
stray flag in one build path silently breaks golden-vector comparison, and the failure is invisible until a
diff appears months later. (b) *double* — same fragility plus 2× the memory traffic on a memory-bound
front end.

---

## R3 — Replay determinism requires virtualising the acquisition scheduler

**Decision**: acquisition runs off-thread live, but in replay it runs **synchronously charged a simulated
budget**, completing at the frame index it would have completed at live (from a measured cost model
recorded in the run's config).

**Rationale**: this is the non-obvious half of replay parity. Frame source and clock virtualisation are
easy; a background acquire that takes ~100 ms wall-clock lands at a *different frame index* when replay runs
10× faster on the dev box, and from that point the two runs diverge irrecoverably. Anything comparing a
replay to a live run — every golden vector, every A/B — depends on this.

**Alternatives**: (a) *let replay diverge and compare statistically* — rejected: it destroys byte-comparable
golden vectors, which are the cheapest regression gate available. (b) *run replay in real time* — rejected:
throws away the ~10× dev-box speedup that makes envelope sweeps affordable.

---

## R4 — Fuse the front end into one pass per L1-sized tile

**Decision**: unpack → spatial high-pass → 2×2/4×4 reduce are **fused into a single pass** over tiles sized
to fit A53's 32 KB L1D (≈64×64 working set), not three passes over the frame.

**Rationale**: the front end is **memory-bound**, not compute-bound — 64 Mpx/s streaming at 250 fps. A naive
three-pass implementation costs 3× the memory traffic and is the difference between ~20 % and ~60 % of a
core. This is a structure decision, not an optimisation to apply later; retrofitting fusion means rewriting
the kernels.

**Alternatives**: *separate, individually-testable passes* — rejected on cost, but the testing concern is
real and is answered instead by testing each fused kernel against a scalar reference that *is* written as
separate passes (R11).

---

## R5 — NEON throughput budget on Cortex-A53: assume 1–2 GMAC/s per core

**Decision**: size all compute estimates at **1–2 GMAC/s int16 per A53 core**, not the paper peak.

**Rationale**: the A53 is in-order, dual-issue, and issues 128-bit ASIMD ops over two cycles, so the
theoretical ~5.6 GMAC/s at 1.4 GHz is unreachable once loads/stores are counted. The spec's §10 budget
already assumes this band; recording it here stops a later "but the datasheet says" argument. Consequence:
cold full-field acquire is **60–120 ms per pass**, not the ~30 ms once claimed in the beacon-receiver
README — which is why acquisition is threaded and why detect-first exists.

**Alternatives**: none — this is a measurement to be confirmed early (see quickstart's kernel bench), not a
choice.

---

## R6 — Threading: four pinned roles, SPSC rings, no locks in the hot path

**Decision**: core 0 OS + libcamera; core 1 capture + fused front end (`SCHED_FIFO`); core 2 bank/track
loop; core 3 acquisition + recorder writer. Single-producer/single-consumer ring buffers with atomics.
`mlockall`, no allocation after init, CPU governor `performance`, heatsink on the 3A+.

**Rationale**: the front end has a hard 4 ms budget and the recorder writes ~64–116 MB/s; both are
jitter-generators for the other. Pinning makes the interference analysable. The 3A+ throttles at 80 °C and
sustained capture already pegs a core — that is a measured 031 fact, not a precaution.

**Alternatives**: (a) *a thread pool / work-stealing* — rejected: unpredictable placement defeats the
deadline analysis for no throughput gain at this scale. (b) *everything on one thread* — rejected: cold
acquire alone is 60–120 ms and would blow the frame budget by 20×.

---

## R7 — Three build paths; the WSL2 cross path needs only a sysroot

**Decision**: (1) Pi native — the always-works field fallback; (2) dev-box native aarch64 — tests, oracle,
scoring; (3) **WSL2 x86_64 → aarch64** via `gcc-aarch64-linux-gnu` + a CMake toolchain file + a sysroot
rsync'd from the Pi. `core/` + `tools/` + `tests/` cross with no sysroot at all.

**Rationale**: field updates are a stated requirement and the field host is WSL2 on x86_64. Because `core/`
has zero dependencies, only `io/`+`app/` ever need the sysroot — so the hard part of cross-compiling
touches the smallest, least-changing part of the tree.

**Alternatives**: (a) *qemu-user-static + an arm64 container* — viable and needs no sysroot management, but
slow and adds a container dependency at the field; kept as documented fallback. (b) *build on the Pi only*
— the fallback path, retained, but a 3A+ with 512 MB is slow enough that it is a poor primary.

---

## R8 — libcamera as an application, not a pipe

**Decision**: `beacon_trackd` and `beacon_record` are libcamera applications (Request-based, DMA buffers,
per-frame metadata, request-level exposure/gain control). The `rpicam-raw` pipe survives **only** as a
replay source and for ad-hoc captures.

**Rationale**: decided in spec §6. Three things are unobtainable through a pipe and all three are
load-bearing: exact per-frame exposure/gain (so the sample series can be normalised and **AGC becomes
transparent to the correlator**), control changes without restarting capture (the 031 `--agc` dumps the ring
on every change), and zero-copy buffers.

**Alternatives**: *keep the pipe and accept restarts* — rejected in the spec; recorded here because it is
the fallback if libcamera bring-up on Trixie proves worse than expected. Bring-up risk is the reason
`beacon_record` (spec §7.1) is the **first** thing built: it exercises the whole libcamera path with no
tracker attached.

---

## R9 — Recording container: framed, versioned, self-describing

**Decision**: a header with magic + format version + sensor mode + build ID + config hash, followed by
length-prefixed frame records carrying `t_us`, exposure, gain and sequence. Preallocated file, `O_DIRECT`,
large aligned writes.

**Rationale**: Constitution V applies directly — this artifact flows between the Pi, the dev box and every
analysis tool, so it carries an explicit version and readers **fail loudly** on mismatch. Build ID + config
hash answer the question field iteration guarantees will be asked: *which binary produced this afternoon's
data?* Preallocation and aligned writes are what keep IO jitter off the §11.1 deadline.

**Alternatives**: (a) *headerless raw, geometry implied by filename* — the 031 status quo; rejected, it
already caused mode-confusion during 031 captures. (b) *an existing container (MP4/MKV/CDR)* — rejected:
all assume encoded video; raw Bayer-less 8-bit mono at 453 fps with custom metadata fits none of them.

---

## R10 — Two-clock synchronisation: optical fiducial for offset, MSP for drift

**Decision**: both, per spec §7.1.2 — an LED in-frame pulsed by an INAV output (one-frame flash every ~5 s)
for frame-exact offset removal, plus MSP over UART FC→Pi timestamped on arrival for drift fitting.

**Rationale**: the Pi's `CLOCK_MONOTONIC` and INAV's blackbox clock are untied, and 10–50 ppm gives 1–5 ms
drift per 100 s sortie — marginal against a 4 ms frame. The optical path needs **zero electrical
integration** between the two systems and self-documents inside data already being kept. The MSP reader is
the same link §2.3's AHRS feed-forward needs, so it pays twice.

**Alternatives**: (a) *GPIO trigger (`ov9281_trigger`, GPIO23)* — harder-wired, and the right answer if
frame-exact **phase locking** is ever needed rather than frame-exact alignment; documented, not built.
(b) *MSP alone* — insufficient: gives ~ms alignment but no hard fiducial to anchor it.

---

## R11 — Testing: scalar reference first, then NEON, then golden vectors

**Decision**: every kernel is written **scalar first** (as separate, obvious passes), tested for behaviour,
then reimplemented fused/NEON and tested for **bit-exact equivalence** against the scalar reference. The
integrated tracker is gated by golden vectors: a fixed clip in, a byte-compared record stream out.

**Rationale**: Constitution I, made cheap. The scalar reference is small, obviously correct, and permanent —
it is the oracle for every future SIMD change, and it costs nothing to keep because it is also the
portable fallback build. Bit-exactness is only meaningful because of R2.

**First golden vector — bootstrapping problem and its answer**: the first golden cannot come from the C
tracker. It comes from `beacon_track.py` on a recorded clip during 042-B, checked by hand against the
offline oracle; once the C tracker reproduces the physically-correct answer, **its** output becomes the
golden and the Python retires (Constitution III).

---

## R12 — Prototype acquisition in Python, against replay, before writing C

**Decision**: acquisition (blink-detect → proto-track RANSAC → multi-rate decode) is prototyped in
NumPy against **recorded clips**, not live, before any C is written. Constitution I's research-spike
exemption is invoked, narrowly: nothing from the prototype is promoted.

**Rationale**: acquisition is the highest-uncertainty algorithm in the feature and the one most likely to be
rewritten three times. NumPy iteration is minutes; C iteration is hours. Working against recorded clips
rather than live is what makes "did that change help?" answerable at all — the same reason the harness comes
before the algorithm.

**Alternatives**: *write it in C first* — rejected: it front-loads the most expensive implementation onto
the least-settled design, which is exactly backwards.
