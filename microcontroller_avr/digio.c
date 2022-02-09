/**
 * digio.c
 */

#include "digio.h"


void DigIO_init(void) {
    // Nothing really required here :-)
}

void DigIO_REGE_setDirection(uint8_t bit, uint8_t dir) {
  uint8_t mask = 0x01 << bit; // we first generate a mask for both function. it only has 1 on the specific index of the register and all others are 0
    if (dir)
      DDRE |= mask;   // we work on the datadirection register E if we want to set the direction,  if we put 1 we tell arduino to use it as an output  // a way to write DDRE = DDRE|mask bitwise or operation
    else
        DDRE &= ~mask;
}

void DigIO_REGE_setValue(uint8_t bit, uint8_t dir) {
    uint8_t mask = 0x01 << bit;
    if (dir)
        PORTE |= mask;
    else
        PORTE &= ~mask;
}
