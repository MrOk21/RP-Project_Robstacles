/**
 * @file pid_motor_example.cpp
 */

#include "digio.h"
#include "pwm.h"
#include "timer.h"
#include "encoder.h"
#include "my_uart.h"
#include "motor.h"
#include <avr/interrupt.h>
#include <stdio.h>
#include <string.h>


// Can be changed, but do you really need to ? 
static const float DT_MS = 10;          // ms
static const float DT_S = DT_MS / 1000; // s

static const uint8_t BIT_REGE_DIRA = 4;
static const uint8_t BIT_REGE_DIRB = 5;
static const float KP = 15.;
static const float KD = 3.;
static const float KI = 1.;// some static values that works good

// Made for you who use arduino often :)
Motor motor(BIT_REGE_DIRA, BIT_REGE_DIRB, KP, KI, KD);

// Don't touch this stuff please
volatile uint8_t timer_int_occurred = 0;
//a volatile variable is excluded from some optimization processes done by the compiler such as
//execution ordering


ISR(TIMER5_COMPA_vect) {
// we design an interrupt routine to handle the 10ms timer that keeps working and at each 10 ms step we
// just switch a variable  to true and it is later used by the main loop to trigger the execution of the  // synchronize loop routine
// we do so in order to verify if every 10ms there is a synchronization on the loop routine
    timer_int_occurred = 1;
    // Spin the motor routine
    motor.spinOnce();
// we also use this function every 10ms; the motor is updated,we check the gain of the feedback loop on the motor.
}



void setup(void) { // we setup all the periferials on arduino
    DigIO_init();
    UART_init();
    TIMER5_init((uint16_t)DT_MS);
    PWM_init();
    encoder_init();
    motor.usePid(true); // and also set the motors to use pid
}

uint8_t rx_buf[64];
uint8_t tx_buf[64];

static int16_t desired_speed = 0;

void sync_loop(void) {
  // we check if we have data coming from PC and we assume that we only sense an
  //integer number that represents the speed
    // if available, read command from serial
    
    if (UART_rxAvailable()) {   
        memset(rx_buf, 0, 64); // set starting point on block memory
        UART_getString(rx_buf);
        desired_speed = atoi((char*)rx_buf);
	// we register this speed and send it to the motor class which will update the control loop
	//and change the behaviour
        // Issue command to motor
        motor.setSpeed(desired_speed);
    }
    sprintf((char *)tx_buf, "des_speed: %d\tmes_speed: %d\terror: %d\n",
        motor.desired_speed(), motor.measured_speed(), (int16_t)motor.error());
    UART_putString(tx_buf);  
}

int main(void) {
    // Arduino like :)
    setup();
    while(1) {
        if (timer_int_occurred) {
            sync_loop();
            timer_int_occurred = 0;
        }
    }
}
