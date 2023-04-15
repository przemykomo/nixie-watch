#include <avr/io.h>
#include <util/delay.h>
#include "i2c-master.h"

#define SHIFT_CLOCK (1 << PB4)
#define SHIFT_DATA (1 << PB3)

#define CLOCK_DELAY 20

#define SHIFT_DIRECTION_PORT DDRB
#define SHIFT_PORT PORTB

#define MAX_DIGIT 11


#define RTC_ADDR 0b1101000
const uint8_t addr = 0b1101000;

void rtc_init() {
    _delay_ms(1000);
    i2c_master_init();
    /*
    i2c_master_start_condition();
    i2c_master_write_byte(0xD0);
    i2c_master_write_byte(0x07); // address of the control register
    i2c_master_write_byte(0b00010001);
    i2c_master_stop_condition();*/
    uint8_t data = 0b00010001;
    i2c_master_write(addr, 0x07, &data, 1);
}
/*
void rtc_set_time(uint8_t h, uint8_t m, uint8_t s) {
    i2c_master_start_condition();
    i2c_master_write_byte(0xD0);
    i2c_master_write_byte(0x00);
    i2c_master_write_byte(s);
    i2c_master_write_byte(m);
    i2c_master_write_byte(h);
    i2c_master_stop_condition();
}*/

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

    rtc_init();
    /*
    rtc_set_time(0x00, 0x00, 0x00);
    uint8_t seconds = 0;
    uint8_t minutes = 0;
    uint8_t hours = 0;

    while (1) {
        _delay_ms(4000);
        i2c_master_start_condition();
        i2c_master_write_byte(0xD0);
        i2c_master_write_byte(0x00);
        i2c_master_start_condition();
        i2c_master_write_byte(0xD1);
        i2c_master_read_byte(&seconds, 0);
        i2c_master_read_byte(&minutes, 0);
        i2c_master_read_byte(&hours, 1);
        i2c_master_stop_condition();

        //write(seconds & 0xF, seconds & 0xF);

        for (uint8_t i = 0; i < (minutes & 0b00001111); i++) {
            _delay_ms(200);
            SHIFT_PORT |= SHIFT_CLOCK;
            _delay_ms(200);
            SHIFT_PORT &= ~SHIFT_CLOCK;
        }
    }*/
    while (1);

    return 0;
}
