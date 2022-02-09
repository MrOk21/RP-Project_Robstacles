/**
 * motor.cpp
 */

#include "motor.h"
extern "C" {
#include "encoder.h"
#include "pwm.h"
#include <math.h>
} // get external files in C to avoid compiler problems, in the header it is made compatible with cpp

template <typename T>
static T clamp(T v, T value){
  if (v>value)
    return value;
  if (v<-value)
    return -value;
  return v;  // useful function to limit a value in a specific simmetrical range, used to bound the error or the velocity input
}

Motor::Motor(uint8_t bit_rege_dira_, uint8_t bit_rege_dirb_, float kp_, float ki_, float kd_) : 
  _dira(bit_rege_dira_), _dirb(bit_rege_dirb_), _kp(kp_), _ki(ki_), _kd(kd_) {

  // set _dira and dirb as DIGIO output pins
  
  DigIO_REGE_setDirection(_dira,1);
  DigIO_REGE_setDirection(_dirb,1);
  
}

/**
 * @brief Enable or disable use of PID internal controller
 * 
 * @param pid_en_ 0 for PWM open loop control, 1 for PID controller
 */
void Motor::usePid(uint8_t pid_en_) {  // it tells the class to use the pid controller, variables defined in the header, it passes true or false and then uses it in spin once function
  _pid_en = pid_en_;
}

/**
 * @brief Set the desired velocity of the motor
 * If PID controller is enabled, issue desired ticks/interval, else
 * issue desired PWM
 * 
 * @param desired_speed_ 
 */
void Motor::setSpeed(int16_t desired_speed_) { // passes a value and stores it in its private space
  _des_speed = desired_speed_;
}

/**
 * @brief Both compute the correction control and interface with
 * the DIGIO/PWM modules to control the joint.
 * Should be used in a time-invariant loop (TIMER5 is handy)
 */
void Motor::spinOnce(void) { // spinOnce is hereditated by the class Motor
  uint16_t  encoder_prev_measurement  = _encoder_measurement;
  _encoder_measurement = encoder_read();
  
  //Compute measurement speed
  
  _mes_speed = (int16_t)(_encoder_measurement - encoder_prev_measurement);
  // cast it to a signed integer since we may be going in the other direction
  
  if (_pid_en) {
    // Compute previous error and current error
    
    _error_prev = error; // we want to take the previous error and compute the current one, we are working in discrete space
    _error= _mes_speed - _des_speed;
    
    //Clamp error with ramp_step maximum time
    _error= clamp<float>(_error,_ramp_step);
    //Compute integral component by accumulating ki*error
    //And clamp the accumulator between +-255
    
    _ei_acc = clamp <float>(_ei_acc + _ki*error, 255.);
    //anti wind-up: if the error is higher than the range, or lower, we bring it back in the range, we prevent shooting on the output controller
    
    //Compute derivate term
    
    float p_term= _kp * _error;
    float d_term= _kd *(_error- _error_prev);
    
    //Clamp the output between +-255
    
    float pid_output = clamp <float>(p_term + d_term + _ei_acc, 255.); //output of pid obtained summing all the terms and clamping them
    output_pwm = int16_t(pid_output);
  }
  else{
    _output_pwm = _des_speed; // if pid disabled we just take the desired speed as output

  }
  //Issue control command to the hardware
  if (_output_pwm < 0 ){ // we first check the sign of the output, according to it we select a direction
    DigIO_REGE_setValue(_dira,1);
    DigIO_REGE_setValue(_dirb;0);
  }
  else{
    DigIO_REGE_setValue(_dira,0);
    DigIO_REGE_setValue(_dirb,1);
  }
  PWM setDutyCycle(abs(_output_pwm)); // we finally pass the pwm signal (abs value) to the pwm module which will power up the real system
  return;
}

// the main idea for our library is that we have a set of functions that we use to give ins for spin method, we dont expect them to do any work, we leave it to the spinonce method 
