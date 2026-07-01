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
//   'R'      reset to nominal (200 Hz, no corruption, no dropout)
#include "config.h"
#include "gold_codes.h"
#include <avr/interrupt.h>

static volatile uint8_t  chip = 0;
static volatile uint8_t  dim_next = 0, sync_next = 0;    // precomputed for the NEXT overflow
static volatile uint8_t  corrupt_n = 0, dropout_n = 0;   // perturbation knobs (chips)
static volatile uint32_t corrupt_mask = 0;               // error positions for the current code word
static volatile uint16_t lfsr = 0xACE1;                  // corruption PRNG

int main(void) {
    // Run the core at the full 20 MHz: default main-clock prescaler is /6, disable it (CCP-protected write).
    CCP = CCP_IOREG_gc;
    CLKCTRL.MCLKCTRLB = 0;                                   // PEN=0 -> 20 MHz (source = OSC20M)

    DIM_PORT.DIRSET  = BM(DIM_PIN);
    SYNC_PORT.DIRSET = BM(SYNC_PIN);
    DIAG_PORT.DIRSET = BM(DIAG_PIN);

    // USART0 command link, 115200 8N1 on the ALTERNATE route PA1(TXD)/PA2(RXD) (default PB2/PB3 not bridged by
    // the XNANO mEDBG CDC). TXD enabled + PA1 output for the RX-echo/heartbeat DIAG.
    PORTMUX.CTRLB |= PORTMUX_USART0_bm;                      // USART0 -> PA1/PA2
    PORTA.DIRSET = BM(1);                                    // PA1 = TXD out
    USART0.BAUD  = UART_BAUD_REG;
    USART0.CTRLA = USART_RXCIE_bm;                           // RX-complete interrupt
    USART0.CTRLB = USART_RXEN_bm | USART_TXEN_bm;            // enable RX + TX

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

    // pre-computed outputs FIRST -> constant-latency (measurement-grade) DIM edge
    if (dim_next)  DIM_PORT.OUTSET  = BM(DIM_PIN);  else DIM_PORT.OUTCLR  = BM(DIM_PIN);
    if (sync_next) SYNC_PORT.OUTSET = BM(SYNC_PIN); else SYNC_PORT.OUTCLR = BM(SYNC_PIN);

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
        }
        op = 0;
    } else if (b == 'F' || b == 'C' || b == 'D') {
        op = b;                                            // value-taking opcode
    } else if (b == 'R') {                                 // reset to nominal
        TCA0.SINGLE.PERBUF = (uint16_t)(TCA_TOP - 1u); corrupt_n = 0; dropout_n = 0;
    }
}
