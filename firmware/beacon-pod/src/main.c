// Beacon emitter (E1b): 200 Hz Gold code (N=31 CODE0) on DIM + a code-epoch SYNC pulse for scope triggering.
// One 200 Hz TCA0 overflow ISR walks the code onto DIM; SYNC is HIGH during chip 0 (like the FPGA sync_pin).
#include "config.h"
#include "gold_codes.h"
#include <avr/interrupt.h>

static volatile uint8_t chip = 0;

int main(void) {
    // Run the core at the full 20 MHz: default main-clock prescaler is /6, disable it (CCP-protected write).
    CCP = CCP_IOREG_gc;
    CLKCTRL.MCLKCTRLB = 0;                                   // PEN=0 -> prescaler off -> 20 MHz (source = OSC20M)

    DIM_PORT.DIRSET  = BM(DIM_PIN);
    SYNC_PORT.DIRSET = BM(SYNC_PIN);
    DIAG_PORT.DIRSET = BM(DIAG_PIN);

    // TCA0 -> exactly 200 Hz: F_CPU/16 / 6250 = 200.000 Hz
    TCA0.SINGLE.PER     = (uint16_t)(TCA_TOP - 1);
    TCA0.SINGLE.INTCTRL = TCA_SINGLE_OVF_bm;
    TCA0.SINGLE.CTRLA   = TCA_SINGLE_CLKSEL_DIV16_gc | TCA_SINGLE_ENABLE_bm;

    sei();
    for (;;) { }                                            // everything happens in the ISR
}

ISR(TCA0_OVF_vect) {
    TCA0.SINGLE.INTFLAGS = TCA_SINGLE_OVF_bm;               // ack

    // drive this chip's code bit (MSB-first, matching the FPGA)
    if ((GOLD_CODE0 >> (GOLD_N - 1u - chip)) & 1u) DIM_PORT.OUTSET = BM(DIM_PIN);
    else                                           DIM_PORT.OUTCLR = BM(DIM_PIN);

    // SYNC HIGH for the duration of chip 0 = code epoch (scope trigger); DIAG heartbeat once per code word
    if (chip == 0) { SYNC_PORT.OUTSET = BM(SYNC_PIN); DIAG_PORT.OUTTGL = BM(DIAG_PIN); }
    else           { SYNC_PORT.OUTCLR = BM(SYNC_PIN); }

    if (++chip >= GOLD_N) chip = 0;
}
