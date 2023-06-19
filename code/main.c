#include "avr/io.h"
#include <util/delay.h>
#include <stdbool.h>
//#include "i2c-master.h"

#define CLOCK_DELAY 20
#define CLOCK_DELAY_FAST 1

#if defined(__AVR_ATtiny412__)
enum {PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7};
#define SHIFT_CLOCK (1 << PA6)
#define SHIFT_DATA (1 << PA3)

#define SHIFT_DIRECTION_PORT VPORTA.DIR
#define SHIFT_PORT VPORTA.OUT

#define BUTTON (1 << PA7)
#define BUTTON_DIRECTION_PORT VPORTA.DIR
#define BUTTON_PORT VPORTA.IN
#else
#define SHIFT_CLOCK (1 << PB4)
#define SHIFT_DATA (1 << PB3)

#define SHIFT_DIRECTION_PORT DDRB
#define SHIFT_PORT PORTB

#define BUTTON (1 << PB1)
#define BUTTON_DIRECTION_PORT DDRB
#define BUTTON_PORT PINB
#endif

#define MAX_DIGIT 11
#define LHDP 10
#define RHDP 11



#define BAUDRATE 4800
#define MYUBRR F_CPU / 16 / BAUDRATE - 1



int last_second = MAX_DIGIT;

// Since the argument to _delay_ms has to be known at compile time, I just duplicated the entire write function.
void write_fast(int first, int second) {
    SHIFT_PORT &= ~SHIFT_CLOCK; // set up low clock

    // move the last digit enough so that it disappears when the final number is displayed
    for (int i = 0; i < (MAX_DIGIT - last_second - first); i++) {
        SHIFT_PORT |= SHIFT_CLOCK;
        _delay_ms(CLOCK_DELAY_FAST);
        SHIFT_PORT &= ~SHIFT_CLOCK;
        _delay_ms(CLOCK_DELAY_FAST);
    }

    last_second = second;

    SHIFT_PORT |= SHIFT_DATA; // set the bit on to shift
    _delay_ms(CLOCK_DELAY_FAST);
    SHIFT_PORT |= SHIFT_CLOCK; // data is shifted on the rising edge
    _delay_ms(CLOCK_DELAY_FAST);
    
    SHIFT_PORT &= ~SHIFT_CLOCK; // data is saved on the falling edge
    SHIFT_PORT &= ~SHIFT_DATA; // disable the bit to shift
    _delay_ms(CLOCK_DELAY_FAST);

    // move the first bit by the difference between first and second bit placement
    for (int i = 0; i < (first + MAX_DIGIT - second); i++) {
        SHIFT_PORT |= SHIFT_CLOCK;
        _delay_ms(CLOCK_DELAY_FAST);
        SHIFT_PORT &= ~SHIFT_CLOCK;
        _delay_ms(CLOCK_DELAY_FAST);
    }

    SHIFT_PORT &= ~SHIFT_CLOCK; // set up low clock
    SHIFT_PORT |= SHIFT_DATA; // set the bit on to shift
    _delay_ms(CLOCK_DELAY_FAST);
    SHIFT_PORT |= SHIFT_CLOCK; // data is shifted on the rising edge
    _delay_ms(CLOCK_DELAY_FAST);
    
    SHIFT_PORT &= ~SHIFT_CLOCK; // data is saved on the falling edge
    SHIFT_PORT &= ~SHIFT_DATA; // disable the bit to shift
    _delay_ms(CLOCK_DELAY_FAST);

    for (int i = 0; i < second; i++) {
        SHIFT_PORT |= SHIFT_CLOCK;
        _delay_ms(CLOCK_DELAY_FAST);
        SHIFT_PORT &= ~SHIFT_CLOCK;
        _delay_ms(CLOCK_DELAY_FAST);
    }
}

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

void write_time() {
    // For some reason when I try to read multiple bytes at the same time I get invalid data.
    // So I decided not to handle the 59 -> 0 minutes edge case.
    uint8_t hours;
    uint8_t minutes;
    //i2c_master_read(0x68, 0x02, &hours, 1);
    //i2c_master_read(0x68, 0x01, &minutes, 1);a

    TWI0.MADDR = 0x68 << 1;
    while (!(TWI0.MSTATUS & (1 << 6)));
    TWI0.MDATA = 0x02;
    while (!(TWI0.MSTATUS & (1 << 6)));
    // stop condition here?
    TWI0.MADDR = (0x68 << 1) | 1;
    while (!(TWI0.MSTATUS & (1 << 7)));
    minutes = TWI0.MDATA;
    while (!(TWI0.MSTATUS & (1 << 7)));
    TWI0.MCTRLB |= 1 << 2;
    hours = TWI0.MDATA;
    TWI0.MCTRLB &= ~(1 << 2);

    write(hours & 0x0F, (hours & 0xF0) >> 4);
    _delay_ms(1000);
    write(minutes & 0x0F, (minutes & 0xF0) >> 4);
    _delay_ms(1000);
    write(RHDP, LHDP);
}

bool last_button_state = false;

int main() {

    // Set the SPI nixie driver pins to output
    SHIFT_DIRECTION_PORT |= SHIFT_CLOCK | SHIFT_DATA;

    BUTTON_DIRECTION_PORT &= ~BUTTON;
    
    //i2c_master_init();
    TWI0.MBAUD = MYUBRR;
    TWI0.MCTRLA |= (1 << 4) | (1 << 1) | (1 << 0);
    TWI0.MCTRLB &= ~(1 << 2);

    // If the button is pressed while the circuit gets powered on, then let the user set the hour.
    if (BUTTON_PORT & BUTTON) {
        while (BUTTON_PORT & BUTTON);
        _delay_ms(50);
        uint8_t hours = 0;
        uint8_t minutes = 0;

        write(hours % 10, hours / 10);
        
        for (int i = 0; i < 500; i++) {
            _delay_ms(10);
            if (BUTTON_PORT & BUTTON) {
                while (BUTTON_PORT & BUTTON);
                _delay_ms(50);
                i = 0;
                hours++;
                hours %= 24;
                write_fast(hours % 10, hours / 10);
            }
        }

        write(minutes % 10, minutes / 10);
        
        for (int i = 0; i < 500; i++) {
            _delay_ms(10);
            if (BUTTON_PORT & BUTTON) {
                while (BUTTON_PORT & BUTTON);
                _delay_ms(50);
                i = 0;
                minutes++;
                minutes %= 60;
                write_fast(minutes % 10, minutes / 10);
            }
        }

        uint8_t data[] = {
            0x00, // seconds
            ((minutes / 10) << 4) | (minutes % 10),
            ((hours / 10) << 4) | (hours % 10),
            0x03, // day of the week
            0x19, // day of the month
            0x04, // month
            0x23, // year
            0b00000000
        };
        //i2c_master_write(0x68, 0x00, data, 8);
        TWI0.MADDR = 0x68 << 1;
        while (!(TWI0.MSTATUS & (1 << 6)));
        TWI0.MDATA = 0x00;

        for (int i = 0; i < 8; i++) {
            while (!(TWI0.MSTATUS & (1 << 6)));
            TWI0.MDATA = data[i];
        }

        while (!(TWI0.MSTATUS & (1 << 6)));
        TWI0.MCTRLB |= 0x3;
    }

    //write(RHDP, LHDP);
    write_time();

    while (1) {
        _delay_ms(10);

        bool button_state = (BUTTON_PORT & BUTTON);
        if (button_state != last_button_state && button_state) {
            _delay_ms(50);
            button_state = (BUTTON_PORT & BUTTON); // debouncing
            if (button_state) {
                write_time();
            }
        }

        last_button_state = button_state;
    } 

    return 0;
}
