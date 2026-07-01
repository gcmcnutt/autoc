// Pin map + clock/timing for the beacon emitter. Signals are assigned for the ATtiny412 TARGET; the ATtiny416
// XNANO eval shares PORTA, so DIM/SYNC land on the SAME physical pins on both parts (only the diagnostic LED
// differs — the XNANO's on-board LED0 is on PB5). Change a signal here, both builds follow.
#pragma once
#include <avr/io.h>

// ---- clock / timing (nominal internal osc; OSCCFG fuse = 20 MHz, verified 0x02) ----
#define F_CPU_HZ         20000000UL      // core clock after main() disables the default /6 prescaler
#define CHIP_RATE_HZ     200UL           // Gold-code chip rate
#define TCA_PRESCALE     16UL            // TCA0 clock = F_CPU / 16
#define TCA_TOP          (F_CPU_HZ / TCA_PRESCALE / CHIP_RATE_HZ)   // counts/chip = 6250  -> PER = TOP-1
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
