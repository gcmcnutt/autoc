// Beacon emitter (E1b): 200 Hz Gold code (N=31 CODE0) on DIM + a code-epoch SYNC pulse for scope triggering.
//
// TIMING DISCIPLINE (so the StepFPGA measures the OSCILLATOR, not the firmware):
//   * Rate is hardware-timed: each chip = exactly TCA_TOP (6250) ticks of F_CPU/16 -> 200.000 Hz. The average
//     rate is immune to ISR latency / added code (a constant offset cancels period-to-period), and won't drift
//     as long as no code holds interrupts off > 1 chip (5 ms) or overruns the ISR (ours is ~20 cyc of 100000).
//   * Edges are DOUBLE-BUFFERED: the next chip's pin states are computed one ISR ahead and applied at the very
//     TOP of the ISR (DIM first), before any work -> constant, data-independent latency from the overflow, so
//     edge jitter is ~1 cycle (~50 ns @ 20 MHz), below the FPGA's 12 MHz (83 ns) counter resolution.
#include "config.h"
#include "gold_codes.h"
#include <avr/interrupt.h>

static volatile uint8_t chip = 0;
static volatile uint8_t dim_next  = 0;      // pin states precomputed for the NEXT overflow
static volatile uint8_t sync_next = 0;

int main(void) {
    // Run the core at the full 20 MHz: default main-clock prescaler is /6, disable it (CCP-protected write).
    CCP = CCP_IOREG_gc;
    CLKCTRL.MCLKCTRLB = 0;                                   // PEN=0 -> prescaler off -> 20 MHz (source = OSC20M)

    DIM_PORT.DIRSET  = BM(DIM_PIN);
    SYNC_PORT.DIRSET = BM(SYNC_PIN);
    DIAG_PORT.DIRSET = BM(DIAG_PIN);

    // seed chip-0 outputs (bit N-1 = chip 0; SYNC high on chip 0)
    dim_next  = (GOLD_CODE[CODE_ID] >> (GOLD_N - 1u)) & 1u;
    sync_next = 1u;

    // TCA0 -> exactly 200 Hz: F_CPU/16 / 6250 = 200.000 Hz
    TCA0.SINGLE.PER     = (uint16_t)(TCA_TOP - 1u);
    TCA0.SINGLE.INTCTRL = TCA_SINGLE_OVF_bm;
    TCA0.SINGLE.CTRLA   = TCA_SINGLE_CLKSEL_DIV16_gc | TCA_SINGLE_ENABLE_bm;

    sei();
    for (;;) { }                                            // everything happens in the ISR
}

ISR(TCA0_OVF_vect) {
    TCA0.SINGLE.INTFLAGS = TCA_SINGLE_OVF_bm;               // ack

    // --- apply pre-computed outputs FIRST: DIM edge = constant latency from the overflow (measurement-grade) ---
    if (dim_next)  DIM_PORT.OUTSET  = BM(DIM_PIN);  else DIM_PORT.OUTCLR  = BM(DIM_PIN);
    if (sync_next) SYNC_PORT.OUTSET = BM(SYNC_PIN); else SYNC_PORT.OUTCLR = BM(SYNC_PIN);

    // --- then advance + compute NEXT chip (its execution time does NOT move the edges above) ---
    if (++chip >= GOLD_N) { chip = 0; DIAG_PORT.OUTTGL = BM(DIAG_PIN); }   // heartbeat once per code word
    dim_next  = (GOLD_CODE[CODE_ID] >> (GOLD_N - 1u - chip)) & 1u;
    sync_next = (chip == 0);
}
