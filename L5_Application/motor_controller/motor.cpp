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
#include "_can_dbc/generated_can.h"
#include "can.h"
#include "string.h"

//Global variables for motor control
PWM * MOTOR;
PWM * SERVO;
int rpm_count;
bool system_started;

//Default initialize motor/steer pwm at 100 Hz
//Initialize the desired PWM
//These functions allow instance ofs PWM to be accessed
PWM * get_motor_pwm(PWM::pwmType pwm)
{
	static PWM motor(PWM::pwm2, 100);
	motor.set(15.0);
	MOTOR = &motor;
	return &motor;
}

PWM * get_servo_pwm(PWM::pwmType pwm)
{
	static PWM servo(PWM::pwm3, 100);
	servo.set(15.0);
	SERVO = &servo;
	return &servo;
}

//Set speed based on input speed -70 to 70 MPH
//Duty cycle will be 10% to 20%, so set speed accordingly
//TBD: handle reverse speed in steps
void set_speed(float speed)
{
	float duty_delta = 10.0/140.0;
	//BREAK!
	if (speed == 0)
	{
		MOTOR->set(15.0);
	} else if ((speed > 0) || (speed < 0)) {
		MOTOR->set(15 + (speed * duty_delta));
		printf("set motor to %f, duty cycle %f\n", speed, 15 + speed*duty_delta);
	} else {
		speed = 0;
		//ERROR: How did we reach this condition?
	}
}

//Angle can be between 30 & -30 degrees
//Duty cycle is 10 to 20 degrees
void set_angle(float angle)
{
	//Duty cycle/angle * angle -> duty cycle/degree
	float angle_delta = 10.0/60.0;
	float new_duty = 0;
	float duty_default = 15.0;
	//printf("delta is %f, duty default is %f\n", angle_delta, duty_default);

	if (angle == 0)
	{
		SERVO->set(15.0);
	} else if ((angle > 0) || (angle < 0)) {
		new_duty = duty_default + (angle * angle_delta);
		SERVO->set(duty_default + (angle * angle_delta));
		printf("set angle to %f, duty cycle %f\n", angle, new_duty);
	} else {
		angle = 0;
		//ERROR: How did we reach this condition?
	}
}

//Bring the car to a halt at neutral steering position
void stop_car()
{
	MOTOR->set(15.0);
	SERVO->set(15.0);
}

//A falling edge interrupt will trigger each time the Hall sensor
//detects a rotation
//Based on the count of these interrupts, and the delta time,
//we can determine how fast the wheels are spinning.
void rpm_intr_hdlr()
{
	rpm_count++;
}

//This is called in 10 kHz periodic, so every 100ms
//rev/100ms * 1000ms/1s * 60s/1min = RPM
int get_rpm_val()
{
	int rpm;
	rpm = (rpm_count * 1000 * 60) / 100;
	rpm_count = 0;
	if (rpm != 0)
	{
	printf("rpm val is %d\n", rpm);
	}
	return rpm;
}

//Scan for start command from master node
void recv_system_start()
{
	MASTER_CONTROL_t master_can_msg;
	can_msg_t can_msg;
	while (CAN_rx(can1, &can_msg, 0))
	{
		// Form the message header from the metadata of the arriving message
		dbc_msg_hdr_t can_msg_hdr;
		can_msg_hdr.dlc = can_msg.frame_fields.data_len;
		can_msg_hdr.mid = can_msg.msg_id;

		// Attempt to decode the message (brute force, but should use switch/case with MID)
		dbc_decode_MASTER_CONTROL(&master_can_msg, can_msg.data.bytes, &can_msg_hdr);
		if (can_msg.data.bytes[0] == DRIVER_HEARTBEAT_cmd_START)
		{
			printf("recv start\n");
		}
	}
	system_started = 1;
}

void send_heartbeat()
{
	MOTOR_HB_t can_msg;
	can_msg.MOTOR_heartbeat = 0x1;
	dbc_encode_and_send_MOTOR_HB(&can_msg);
}

void update_speed_and_angle()
{
	bool rc = 0;
	MOTOR_UPDATE_t motor_can_msg;
	can_msg_t can_msg;
	while (CAN_rx(can1, &can_msg, 0))
	{
		// Form the message header from the metadata of the arriving message
		dbc_msg_hdr_t can_msg_hdr;
		can_msg_hdr.dlc = can_msg.frame_fields.data_len;
		can_msg_hdr.mid = can_msg.msg_id;

		// Attempt to decode the message (brute force, but should use switch/case with MID)
		rc = dbc_decode_MOTOR_UPDATE(&motor_can_msg, can_msg.data.bytes, &can_msg_hdr);
		if (rc == true)
		{
			set_speed((float)motor_can_msg.MOTOR_speed);
			set_angle((float)motor_can_msg.MOTOR_turn_angle);
		}
	}
}

bool dbc_app_send_can_msg(uint32_t mid, uint8_t dlc, uint8_t bytes[8])
{
    can_msg_t can_msg = { 0 };
    can_msg.msg_id                = mid;
    can_msg.frame_fields.data_len = dlc;
    memcpy(can_msg.data.bytes, bytes, dlc);

    return CAN_tx(can1, &can_msg, 0);
}


