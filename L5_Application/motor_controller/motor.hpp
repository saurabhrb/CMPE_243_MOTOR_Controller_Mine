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
extern int rpm_count;

PWM * get_motor_pwm(PWM::pwmType pwm);
PWM * get_servo_pwm(PWM::pwmType pwm);
//Set speed of car
void set_speed(float speed);
//Turn to certain degree
void set_angle(float angle);

//Read RPM value
void rpm_intr_hdlr();
int get_rpm_val();

//Bring the car to a halt at neutral steering position
void stop_car();



#endif /* L5_APPLICATION_MOTOR_CONTROLLER_MOTOR_HPP_ */
