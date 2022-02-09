/**
 * encoder.c
 */

#include "encoder.h"
#include <avr/io.h>
#include <avr/interrupt.h>

#define PIN_MASK 0x03 // we will be working on the 2 low pins: pins 0 and 1. the equivalent mask is 000000011 = 3 

// Set to volatile to store readings in RAM instead of registers
volatile uint8_t portb_prev_reading;
volatile uint8_t portb_curr_reading;

volatile uint16_t encoder_counts;

static const int8_t _transition_table []= {
    0,  //0000
    1,  //0001
   -1,  //0010
    0,  //0011
   -1,  //0100
    0,  //0101
    0,  //0110
    1,  //0111
    1,  //1000
    0,  //1001
    0,  //1010
   -1,  //1011
    0,  //1100
   -1,  //1101
    1,  //1110
    0   //1111
};

void encoder_init(void) {
    // cli/sei block for avoiding interrupts
    cli();
    // Use port B for the encoders
    DDRB &= ~PIN_MASK; 
    // set pins as input we do an and bit by bit operation in order to clear only the 2 bits that correspond 
    // to the last 2 digits this preserves the status of the existing digits in the port
    PORTB |= PIN_MASK; // enable pull-up resistors on pins
    
    // Install interrupts
    PCICR |= (1 << PCIE0); // Set interrupt on change, looking for PCMSK0
    PCMSK0 |= PIN_MASK; // set PCINT0 to trigger an interrupt on state change
    
    // Initialize encoder variables
    portb_prev_reading = PIN_MASK;
    portb_curr_reading = PIN_MASK;
    encoder_counts = 0;
    sei(); // it reastablishes the interrupt
}

uint16_t encoder_read(void) {
    return encoder_counts;
}

ISR(PCINT0_vect) {
    portb_prev_reading = portb_curr_reading;
    portb_curr_reading = PINB;

    // generate idx for accessing transition_table
    uint8_t idx = (portb_prev_reading & PIN_MASK) << 2 | // left shift operation
        (portb_curr_reading & PIN_MASK);  // we have packed the 2 pins into a number of 4 bits
    encoder_counts += _transition_table[idx]; 
}
// in ISR function we install an interrupt handler on pin change on the 2 pins and whenever the interrupt triggers we know that some pins change so we can call the encoder routine to update the global variable, the counter
