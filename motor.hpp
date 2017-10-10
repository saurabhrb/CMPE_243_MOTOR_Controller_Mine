/*
 * motor.hpp
 *
 *  Created on: Oct 7, 2017
 *      Author: jskow
 */

#ifndef L5_APPLICATION_MOTOR_CONTROLLER_MOTOR_HPP_
#define L5_APPLICATION_MOTOR_CONTROLLER_MOTOR_HPP_

//These are initialized in period_callback period_init
extern PWM *MOTOR;
extern PWM *SERVO;

PWM * get_motor_pwm(PWM::pwmType pwm);
PWM * get_servo_pwm(PWM::pwmType pwm);
void go_forward(float distance);
void go_reverse(float distance);
//Turn to certain degree
void go_left(float angle);

void go_right(float angle);

//Bring the car to a halt at neutral steering position
void stop_car();



#endif /* L5_APPLICATION_MOTOR_CONTROLLER_MOTOR_HPP_ */
