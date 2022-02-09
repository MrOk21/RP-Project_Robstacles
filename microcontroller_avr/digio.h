/**
 * digio.h
 */

#pragma once

#include <stdint.h>
#include <avr/io.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the Digital I/O module acting on 
 * Register E.
 * The pins on Reg.E are:
 *  PE0 : Pin 0   //map of the bits of the register and the pin of arduino
 *  PE1 : Pin 1
 *  PE2 : -
 *  PE3 : Pin 5 (PWM)
 *  PE4 : Pin 2 (PWM)
 *  PE5 : Pin 3 (PWM)
 *  PE6 : -
 *  PE7 : -
 */
void DigIO_init(void);

/**
 * @brief Set the direction of a pin on Register E
 * 
 * @param bit PEx where x is the bit of the target pin.
 * @param dir Direction of the target pin: 0 for input, 1 for output
 */
  void DigIO_REGE_setDirection(uint8_t bit, uint8_t dir); // set a direction of the pin: we either want the pin to receive a data from outside arduino towards inside 

/**
 * @brief Set the value of a pin on Register E
 * 
 * @param bit PEx where x is the bit of the target pin.
 * @param val Value of the target pin:
 *  if pin's direction is 0 (INPUT): 
 *      0 for direct connection, 1 for pull-up resistor enable 
 *  if pin's direction is 1 (OUTPUT): 
 *      0 for LOW, 1 for HIGH
 */
  void DigIO_REGE_setValue(uint8_t bit, uint8_t val); // a function to decide whether to turn the pin on or off

#ifdef __cplusplus
}
#endif
