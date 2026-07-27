// Beacon emitter: 200 Hz Gold code (N=31) on DIM + a code-epoch SYNC pulse, with a USART0 command link so a host
// can perturb the REAL emitter for closed-loop tests against the StepFPGA (frequency / corruption / dropout).
//
// TIMING DISCIPLINE (so the StepFPGA measures the OSCILLATOR, not the firmware):
//   * Rate is hardware-timed: each chip = TCA_TOP ticks of F_CPU/16 -> 200.000 Hz nominal. Frequency retune uses
//     PERBUF (hardware double-buffered) so it takes effect glitch-free at the next overflow.
//   * DIM/SYNC edges are DOUBLE-BUFFERED and applied at the TOP of the ISR, before any work (incl. the command
//     effects), so edge latency stays constant (~1 cycle jitter) regardless of corruption/dropout/UART activity.
//
// Command protocol (bytes on USART0 RX @ 115200):
//   'F' <v>  set chip rate:  PER = TCA_TOP-1 - (v-128)*5   (v=128 nominal 200 Hz, ~±10% over the byte)
//   'C' <n>  corrupt: flip n random chips / code word
//   'D' <n>  dropout: blank the first n chips / code word (DIM held low)
//   'P' <v>  lit-chip pulse width = v/256 of a chip (0 or 255 = full width). Hardware-timed via TCA CMP0:
//            a photon-flux attenuator for link-margin / AGC-response tests (receiver samples catch the lit
//            fraction with probability v/256 -> mean correlation scales ~linearly with v).
//            ⚠ BENCH-ONLY: the production waveform contract is FULL-DUTY chips. The eventual camera receiver
//            integrates a 10-60%-of-frame exposure window that slides through the chip (unsynced clocks);
//            partial-duty pulses would overlap it by a phase-dependent amount -> amplitude beats at the
//            clock-slip rate. Full-duty chips are exposure-phase-immune. (Single-PD @480 Hz doesn't care.)
//   'R'      reset to nominal (200 Hz, no corruption, no dropout, full-width pulses)
#include "config.h"
#include "gold_codes.h"
#include <avr/interrupt.h>

static volatile uint8_t  chip = 0;
static volatile uint8_t  dim_next = 0, sync_next = 0;    // precomputed for the NEXT overflow
static volatile uint8_t  corrupt_n = 0, dropout_n = 0;   // perturbation knobs (chips)
static volatile uint8_t  pulse_w = 0;                    // 'P': lit-chip width = pulse_w/256 chip (0 = full)
static volatile uint32_t corrupt_mask = 0;               // error positions for the current code word
static volatile uint16_t lfsr = 0xACE1;                  // corruption PRNG

// ---- firmware-ADC UVLO (R11): trip -> LEDs dark + POWER_DOWN; recovery = battery pull / POR ----
static uint8_t uvlo_div = 0, uvlo_cnt = 0;               // ISR-only state (sample cadence / debounce)

static void uvlo_shutdown(void) {
    cli();
    DIM_PORT.OUTCLR  = BM(DIM_PIN);  DIM_PORT.DIRCLR  = BM(DIM_PIN);   // release DIM: R2 pull-DOWN owns the node
    SYNC_PORT.OUTCLR = BM(SYNC_PIN); SYNC_PORT.DIRCLR = BM(SYNC_PIN);  //   -> LM3410X in ~80 nA shutdown
    DIAG_PORT.DIRCLR = BM(DIAG_PIN);                                   // release diag LED (off either polarity)
    TCA0.SINGLE.CTRLA = 0; USART0.CTRLB = 0; ADC0.CTRLA = 0;
    PORTA.OUTSET = BM(1);                                // park TX (PA1) idle-HIGH: a low UART line = endless
                                                         // break -> the mEDBG floods 0x00 (seen 2026-07-18)
    _PROTECTED_WRITE(WDT.CTRLA, WDT_PERIOD_OFF_gc);      // else the WDT would reboot-loop us out of sleep
    SLPCTRL.CTRLA = SLPCTRL_SMODE_PDOWN_gc | SLPCTRL_SEN_bm;
    for (;;) __asm__ __volatile__("sleep");              // no wake sources armed: sleep until power is pulled
}

int main(void) {
    // Core at 10 MHz = OSC20M/2 (CCP-protected write). NOT the full 20 MHz: that speed grade needs VDD >= 4.5 V
    // (out of spec on 1S LiPo); 10 MHz is valid 2.7-5.5 V — matches the LM3410X floor (A2-pwr, 2026-07-16).
    CCP = CCP_IOREG_gc;
    CLKCTRL.MCLKCTRLB = CLKCTRL_PDIV_2X_gc | CLKCTRL_PEN_bm; // PEN=1, PDIV=/2 -> 10 MHz

    // Watchdog FIRST (eval check (b): hung firmware -> reset <= 250 ms -> R2 pull-down darkens the string
    // through the reset). Fed once per chip in the TCA ISR.
    _PROTECTED_WRITE(WDT.CTRLA, WDT_PERIOD_256CLK_gc);       // ~0.256 s @ 1 kHz WDT osc

    DIM_PORT.DIRSET  = BM(DIM_PIN);
    SYNC_PORT.DIRSET = BM(SYNC_PIN);
    DIAG_PORT.DIRSET = BM(DIAG_PIN);

    // UVLO ADC: 1.1 V internal ref measured against VDD (result rises as VDD falls; trip > UVLO_ADC_TRIP).
    VREF.CTRLA  = VREF_ADC0REFSEL_1V1_gc;
    ADC0.CTRLC  = ADC_PRESC_DIV16_gc | ADC_REFSEL_VDDREF_gc | ADC_SAMPCAP_bm;   // 625 kHz ADC clk @ 10 MHz
    ADC0.MUXPOS = ADC_MUXPOS_INTREF_gc;
    ADC0.CTRLA  = ADC_ENABLE_bm;                              // 10-bit
    ADC0.COMMAND = ADC_STCONV_bm;                             // throwaway conversion (reference settle)
    while (!(ADC0.INTFLAGS & ADC_RESRDY_bm)) { }
    (void)ADC0.RES;

    // USART0 command link, 115200 8N1 on the ALTERNATE route PA1(TXD)/PA2(RXD) (default PB2/PB3 not bridged by
    // the XNANO mEDBG CDC). TXD enabled + PA1 output for the RX-echo/heartbeat DIAG.
    PORTMUX.CTRLB |= PORTMUX_USART0_bm;                      // USART0 -> PA1/PA2
    PORTA.DIRSET = BM(1);                                    // PA1 = TXD out
    USART0.BAUD  = UART_BAUD_REG;
    USART0.CTRLA = USART_RXCIE_bm;                           // RX-complete interrupt
    USART0.CTRLB = USART_RXEN_bm | USART_TXEN_bm;            // enable RX + TX

    // Boot banner: 'B' + RSTFR hex -> host attributes every reset (PORF=01 BORF=02 EXTRF=04 WDRF=08 SWRF=10
    // UPDIRF=20). Added for the micro-dropout hunt (A2-uvlo-2): transient resets seen near the UVLO threshold.
    uint8_t rf = RSTCTRL.RSTFR;
    RSTCTRL.RSTFR = rf;                                      // write-1-to-clear so each banner is per-event
    while (!(USART0.STATUS & USART_DREIF_bm)) { }  USART0.TXDATAL = 'B';
    while (!(USART0.STATUS & USART_DREIF_bm)) { }  USART0.TXDATAL = "0123456789ABCDEF"[rf >> 4];
    while (!(USART0.STATUS & USART_DREIF_bm)) { }  USART0.TXDATAL = "0123456789ABCDEF"[rf & 0x0F];

    dim_next  = (GOLD_CODE[CODE_ID] >> (GOLD_N - 1u)) & 1u;  // seed chip 0
    sync_next = 1u;

    TCA0.SINGLE.PER     = (uint16_t)(TCA_TOP - 1u);          // exactly 200 Hz
    TCA0.SINGLE.INTCTRL = TCA_SINGLE_OVF_bm;
    TCA0.SINGLE.CTRLA   = TCA_SINGLE_CLKSEL_DIV16_gc | TCA_SINGLE_ENABLE_bm;

    sei();
    for (;;) { }                                            // all work in the ISRs
}

// ---- 200 Hz chip clock: apply this chip's outputs, then compute the next (with any commanded perturbation) ----
ISR(TCA0_OVF_vect) {
    TCA0.SINGLE.INTFLAGS = TCA_SINGLE_OVF_bm;               // ack
    __asm__ __volatile__("wdr");                            // feed the watchdog (chip clock alive)

    // pre-computed outputs FIRST -> constant-latency (measurement-grade) DIM edge
    if (dim_next)  DIM_PORT.OUTSET  = BM(DIM_PIN);  else DIM_PORT.OUTCLR  = BM(DIM_PIN);
    if (sync_next) SYNC_PORT.OUTSET = BM(SYNC_PIN); else SYNC_PORT.OUTCLR = BM(SYNC_PIN);

    // 'P' pulse-shortening: arm CMP0 to cut the lit chip after pulse_w/256 of the period (hardware-timed).
    if (pulse_w && dim_next) {
        TCA0.SINGLE.CMP0 = (uint16_t)(((uint32_t)TCA0.SINGLE.PER * pulse_w) >> 8);
        TCA0.SINGLE.INTFLAGS = TCA_SINGLE_CMP0_bm;
        TCA0.SINGLE.INTCTRL  = TCA_SINGLE_OVF_bm | TCA_SINGLE_CMP0_bm;
    } else {
        TCA0.SINGLE.INTCTRL  = TCA_SINGLE_OVF_bm;
    }

    // UVLO sample every UVLO_SAMPLE_CHIPS chips (non-blocking: read last conversion, kick the next).
    if (++uvlo_div >= UVLO_SAMPLE_CHIPS) {
        uvlo_div = 0;
        if (ADC0.INTFLAGS & ADC_RESRDY_bm) {
            uint16_t res = ADC0.RES;                        // read clears RESRDY
            if (res > UVLO_ADC_TRIP) {                      // VDD below ~3.6 V firmware-set
                if (++uvlo_cnt >= UVLO_TRIP_COUNT) uvlo_shutdown();   // ~500 ms debounced -> dark + sleep
            } else uvlo_cnt = 0;
        }
        ADC0.COMMAND = ADC_STCONV_bm;
    }

    if (++chip >= GOLD_N) {                                 // new code word
        chip = 0;
        DIAG_PORT.OUTTGL = BM(DIAG_PIN);                    // heartbeat
        corrupt_mask = 0;                                  // pick this word's error positions
        for (uint8_t i = 0; i < corrupt_n; i++) {
            lfsr = (uint16_t)((lfsr >> 1) ^ (-(lfsr & 1u) & 0xB400u));
            corrupt_mask |= (1UL << (lfsr % GOLD_N));
        }
    }
    uint8_t bit = (GOLD_CODE[CODE_ID] >> (GOLD_N - 1u - chip)) & 1u;
    if (corrupt_mask & (1UL << chip)) bit ^= 1u;           // corruption: flip chosen chips
    dim_next  = (chip < dropout_n) ? 0u : bit;             // dropout: blank the leading chips
    sync_next = (chip == 0);
}

// ---- 'P' pulse cut: end the lit portion of this chip (OUTCLR harmless if the chip was already dark) ----
ISR(TCA0_CMP0_vect) {
    TCA0.SINGLE.INTFLAGS = TCA_SINGLE_CMP0_bm;
    DIM_PORT.OUTCLR = BM(DIM_PIN);
}

// ---- host command receiver ----
ISR(USART0_RXC_vect) {
    static uint8_t op = 0;                                  // opcode awaiting its value byte
    uint8_t b = USART0.RXDATAL;                             // read clears RXC
    if (USART0.STATUS & USART_DREIF_bm) USART0.TXDATAL = b; // echo the byte back (host ack / baud-error check)
    if (op) {
        switch (op) {
            case 'F': TCA0.SINGLE.PERBUF = (uint16_t)((int16_t)(TCA_TOP - 1u) - ((int16_t)b - 128) * 5); break;
            case 'C': corrupt_n = (b > GOLD_N) ? GOLD_N : b; break;
            case 'D': dropout_n = (b > GOLD_N) ? GOLD_N : b; break;
            case 'P': pulse_w = (b == 255u) ? 0u : b; break;   // 0 or 255 = full width
        }
        op = 0;
    } else if (b == 'F' || b == 'C' || b == 'D' || b == 'P') {
        op = b;                                            // value-taking opcode
    } else if (b == 'R') {                                 // reset to nominal
        TCA0.SINGLE.PERBUF = (uint16_t)(TCA_TOP - 1u); corrupt_n = 0; dropout_n = 0; pulse_w = 0;
    }
}
