#include <avr/io.h>
#include <util/delay.h>

#define SHIFT_CLOCK (1 << PB1)
#define SHIFT_DATA (1 << PB0)

#define CLOCK_DELAY 20

#define SHIFT_DIRECTION_PORT DDRB
#define SHIFT_PORT PORTB

#define MAX_DIGIT 11

int last_second = MAX_DIGIT;

void write(int first, int second) {
    SHIFT_PORT &= ~SHIFT_CLOCK; // set up low clock

    // move the last digit enough so that it disappears when the final number is displayed
    for (int i = 0; i < (MAX_DIGIT - last_second - first); i++) {
        SHIFT_PORT |= SHIFT_CLOCK;
        _delay_ms(CLOCK_DELAY);
        SHIFT_PORT &= ~SHIFT_CLOCK;
        _delay_ms(CLOCK_DELAY);
    }

    last_second = second;

    SHIFT_PORT |= SHIFT_DATA; // set the bit on to shift
    _delay_ms(CLOCK_DELAY);
    SHIFT_PORT |= SHIFT_CLOCK; // data is shifted on the rising edge
    _delay_ms(CLOCK_DELAY);
    
    SHIFT_PORT &= ~SHIFT_CLOCK; // data is saved on the falling edge
    SHIFT_PORT &= ~SHIFT_DATA; // disable the bit to shift
    _delay_ms(CLOCK_DELAY);

    // move the first bit by the difference between first and second bit placement
    for (int i = 0; i < (first + MAX_DIGIT - second); i++) {
        SHIFT_PORT |= SHIFT_CLOCK;
        _delay_ms(CLOCK_DELAY);
        SHIFT_PORT &= ~SHIFT_CLOCK;
        _delay_ms(CLOCK_DELAY);
    }

    SHIFT_PORT &= ~SHIFT_CLOCK; // set up low clock
    SHIFT_PORT |= SHIFT_DATA; // set the bit on to shift
    _delay_ms(CLOCK_DELAY);
    SHIFT_PORT |= SHIFT_CLOCK; // data is shifted on the rising edge
    _delay_ms(CLOCK_DELAY);
    
    SHIFT_PORT &= ~SHIFT_CLOCK; // data is saved on the falling edge
    SHIFT_PORT &= ~SHIFT_DATA; // disable the bit to shift
    _delay_ms(CLOCK_DELAY);

    for (int i = 0; i < second; i++) {
        SHIFT_PORT |= SHIFT_CLOCK;
        _delay_ms(CLOCK_DELAY);
        SHIFT_PORT &= ~SHIFT_CLOCK;
        _delay_ms(CLOCK_DELAY);
    }
}

int main() {

    // Set the SPI nixie driver pins to output
    SHIFT_DIRECTION_PORT |= SHIFT_CLOCK | SHIFT_DATA;

    while (1) {
        for (int i = 0; i < 100; i++) {
            int first = i / 10;
            int second = i % 10;
            write(first, second);
            _delay_ms(700);
        }
    }

    return 0;
}
