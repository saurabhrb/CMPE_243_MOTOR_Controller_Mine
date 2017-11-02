/*
 * motor.hpp
 *
 *  Created on: Oct 7, 2017
 *      Author: jskow
 */
#include "lpc_pwm.hpp"

#ifndef L5_APPLICATION_MOTOR_CONTROLLER_MOTOR_HPP_
#define L5_APPLICATION_MOTOR_CONTROLLER_MOTOR_HPP_

//These are initialized in period_callback period_init
extern PWM *MOTOR;
extern PWM *SERVO;
extern int mps_count;
extern bool system_started;
extern float cur_speed;
extern uint64_t cur_clk;

PWM * get_motor_pwm(PWM::pwmType pwm);
PWM * get_servo_pwm(PWM::pwmType pwm);
//Set speed of car
bool set_speed(float speed);
//Turn to certain degree
void set_angle(float angle);
//Make sure speed is as expected
void check_speed(int count);
//Send speed/battery voltage to Master
void send_feedback();

//Read RPM value
void mps_intr_hdlr();
float get_mps_val();

//Bring the car to a halt at neutral steering position
void stop_car();

//Relevant CAN helper functions for motor controller
void recv_system_start();
void send_heartbeat();
void update_speed_and_angle();


#endif /* L5_APPLICATION_MOTOR_CONTROLLER_MOTOR_HPP_ */
