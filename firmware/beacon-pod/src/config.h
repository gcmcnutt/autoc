// Pin map + clock/timing for the beacon emitter. Signals are assigned for the ATtiny412 TARGET; the ATtiny416
// XNANO eval shares PORTA, so DIM/SYNC land on the SAME physical pins on both parts (only the diagnostic LED
// differs — the XNANO's on-board LED0 is on PB5). Change a signal here, both builds follow.
#pragma once
#include <avr/io.h>

// ---- clock / timing (nominal internal osc; OSCCFG fuse = 20 MHz, verified 0x02) ----
// 10 MHz = OSC20M/2 via the main-clock prescaler (A2-pwr, 2026-07-16): the tinyAVR-1 speed grade for 20 MHz
// requires VDD >= 4.5 V — out of spec across the whole 1S LiPo range. 10 MHz is in-spec 2.7–5.5 V, which
// matches the LM3410X's 2.7 V floor. 200 Hz chip stays EXACT (TCA_TOP = 3125).
#define F_CPU_HZ         10000000UL      // core clock after main() sets the prescaler to /2 (OSC20M/2)
#define CHIP_RATE_HZ     200UL           // Gold-code chip rate
#define TCA_PRESCALE     16UL            // TCA0 clock = F_CPU / 16
#define TCA_TOP          (F_CPU_HZ / TCA_PRESCALE / CHIP_RATE_HZ)   // counts/chip = 3125  -> PER = TOP-1
#if (F_CPU_HZ % (TCA_PRESCALE * CHIP_RATE_HZ)) != 0
#  error "CHIP_RATE_HZ is not an exact divisor of F_CPU/prescale — 200 Hz would not be exact"
#endif

// ---- logical signal -> physical pin (PORTA on BOTH 412 + 416) ----
#define DIM_PORT   PORTA     // code output (baseband, to the LED-driver DIM)      412: PA3
#define DIM_PIN    3
#define SYNC_PORT  PORTA     // code-epoch sync pulse (scope trigger; HIGH in chip 0)  412: PA7
#define SYNC_PIN   7
#if defined(__AVR_ATtiny416__)          // XNANO eval — on-board LED0
#  define DIAG_PORT PORTB
#  define DIAG_PIN  5
#else                                    // 412 production target
#  define DIAG_PORT PORTA
#  define DIAG_PIN  6
#endif

#define BM(pin)   (1u << (pin))

// which Gold code this pod emits: 0 = code A (CODE0), 1 = code B (CODE1). Compile-time for now (later: the
// PA1/PA2 code-select pins per mcu-firmware-contract). The 416 eval emits CODE B for the s4 hardware-in-loop test.
#ifndef CODE_ID
#  define CODE_ID  1
#endif

// USART0 host->emitter command link over the mEDBG CDC (= /dev/ttyACM0). The XNANO wires the CDC to USART0's
// ALTERNATE route TXD=PA1 / RXD=PA2 (NOT the default PB2/PB3), and the mEDBG tri-states these unless the host
// asserts DTR (pyserial does by default). Lets a host script perturb the REAL emitter for closed-loop tests:
// retune frequency, inject bit errors, blank chips (dropout). See main.c for the byte protocol.
#define UART_BAUD      38400UL       // modest rate: solid margin vs the RC-osc baud error (115200 showed MSB flips)
#define UART_BAUD_REG  ((uint16_t)((64UL * F_CPU_HZ) / (16UL * UART_BAUD)))   // async normal mode; = 1041 @ 10 MHz (+0.06%)
