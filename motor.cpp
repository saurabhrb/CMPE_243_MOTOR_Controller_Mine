/*
 * 	This file contains motor controller functions for the Traxxas RC car.
 *  Control is provided for the motor driver and servo motor (left/right).
 *
 * TBD: Go forward/reverse by distance
 * 		Take in angle required for turning left/right
 * 		Use singleton instance to access PWM
 */
#include "lpc_pwm.hpp"
#include "LPC17xx.h"
#include "sys_config.h"
#include "stdio.h"
#include "motor.hpp"

PWM * MOTOR;
PWM * SERVO;

//Default initialize motor/steer pwm at 100 Hz
//Initialize the desired PWM
//These functions allow instance ofs PWM to be accessed
PWM * get_motor_pwm(PWM::pwmType pwm)
{
	static PWM motor(PWM::pwm1, 100);
	motor.set(15.0);
	MOTOR = &motor;
	return &motor;
}

PWM * get_servo_pwm(PWM::pwmType pwm)
{
	static PWM servo(PWM::pwm2, 100);
	servo.set(15.0);
	SERVO = &servo;
	return &servo;
}


//Go forward at desired speed
//TBD: Go forward certain distance?
void go_forward(float distance)
{
	printf("go forward\n");
	MOTOR->set(15.1);
}

//Going in reverse requires a gradual increase
void go_reverse(float distance)
{
	MOTOR->set(14.9);
}

//Turn to certain degree
void go_left(float angle)
{
	SERVO->set(14.9);
}

void go_right(float angle)
{
	SERVO->set(15.1);
}

//Bring the car to a halt at neutral steering position
void stop_car()
{
	MOTOR->set(15.0);
	SERVO->set(15.0);
}

