// Beacon emitter firmware. E1 bring-up = blink LED0 (prove the flash loop before the Gold-code ISR).
// LED0 = PB5 on the ATtiny416 Xplained Nano (active-low; toggling blinks regardless). F_CPU from platformio.ini.
#include <avr/io.h>
#include <util/delay.h>

int main(void) {
    PORTB.DIRSET = PIN5_bm;              // PB5 output
    for (;;) {
        PORTB.OUTTGL = PIN5_bm;         // toggle LED0
        _delay_ms(250);                 // ~2 Hz
    }
}
