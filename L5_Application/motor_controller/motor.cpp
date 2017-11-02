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
#include "io.hpp"
#include "lpc_sys.h"

//Global variables for motor control
PWM * MOTOR;
PWM * SERVO;
int mps_count;
float cur_speed;
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
//Return: 1 if stepping required, 0 if done setting speed
bool set_speed(float speed)
{
	//float duty_delta = 10.0/140.0;
	float duty_delta = 10.0/(67.0);
	float speed_to_set;
	//BREAK!
	if (speed == 0)
	{
		MOTOR->set(15.0);
	} else if (speed > 0) {
		speed_to_set = 15.0 + (speed * duty_delta);
		if (speed_to_set < 15.3)
		{
			speed_to_set = 15.3;
		}
		MOTOR->set(speed_to_set);
		printf("set motor to %f, duty cycle %f\n", speed, 15.0 + speed*duty_delta);
	} else if (speed < 0) {
		speed_to_set = 15.0 + (speed * duty_delta);
		if ((speed_to_set > 14.7) && (speed_to_set < 15.0))
		{
			speed_to_set = 14.7;
		}
		MOTOR->set(speed_to_set);
		printf("set motor to %f, duty cycle %f\n", speed, 15.0 + speed*duty_delta);
	} else {
		speed = 0;
		//ERROR: How did we reach this condition?
	}
	return 0;
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

float prev_speed_val=0;

int total_count = 0;
int old_count = 0;
float mps_val=0;

//If measured speed is not expected speed, then step up/down
//until you reach the desired set speed
void check_speed(int count)
{
	float meas_speed = get_mps_val();
	float delta, step = .3;

	//If speed is within 1mps, then no need to change
	float margin = 1;

	//Check if speed went to 0
	if ((count % 20) == 0)
	{
		if (old_count == total_count)
		{
			mps_val = 0;
		}
	} else if ((count % 10) == 0) {
		old_count = total_count;
	}

	//If current speed is < 0, we are in reverse
	if (cur_speed < 0)
	{
		meas_speed = -meas_speed;
	}
	delta = meas_speed - cur_speed;

	printf("delta speed is %f\n", delta);
	//If delta greater than 0, we want to speed up
	if ((delta < 0) && (delta < -margin))
	{
		prev_speed_val+=step;
		//step = -delta;
		set_speed(prev_speed_val);
	} else if ((delta > 0) && (delta > margin)) {
		prev_speed_val-=step;
		//step = -delta;
		set_speed(prev_speed_val);
	} else {

	}

//	else if ((delta < 0) && (delta < -margin)){
//		//If measured less than current, slow down
//		step = .1;
//		set_speed(meas_speed+step);
//	}
}

uint64_t cur_clk;
uint64_t t_delt = 0;
float circumference = 32.99/100;
//A falling edge interrupt will trigger each time the Hall sensor
//detects a rotation
//Based on the count of these interrupts, and the delta time,
//we can determine how fast the wheels are spinning.
void mps_intr_hdlr()
{
	//printf("saw a wheel spin\n");
	//mps_count++;

	//Every two ticks, check the RPM
	if (mps_count == 2)
	{
		t_delt = sys_get_uptime_us() - cur_clk;
		mps_count = 0;
		//Get time delta
		cur_clk = sys_get_uptime_us();

		mps_val = (2.0 * circumference * (10e+6)) / t_delt;
		mps_val /= 10.0;
	} else {
		//start time
		mps_count++;
	}
	total_count++;
}

//This is called in 10 kHz periodic, so every 100ms
//mps = meters per second
//rev/100ms * 1000ms/1s * circumference (m)= mps
float get_mps_val()
{
	float mps;
	//float circumference = 32.99/100;
	//mps = ((float)mps_count * circumference * 1000.0) / (100.0);
	mps = mps_val;
	//rpm = (rpm_count * 1000 * 60) / 100;
	//mps = ((float)mps_count * circumference);
	//mps_count = 0;
	if (mps != 0)
	{
	printf("mps val is %f\n", mps);
	}
	return mps;
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
		if (can_msg.data.bytes[0] == MASTER_cmd_START)
		{
			stop_car();
			LE.on(1);
			printf("recv start\n");
			system_started = 1;
		}
	}
	// Service the MIA counters
	// successful decoding resets the MIA counter, otherwise it will increment to
	// its MIA value and upon the MIA trigger, it will get replaced by your MIA struct
	//rc = dbc_handle_mia_LAB_TEST(&master_can_msg, 1000);  // 1000ms due to 1Hz
	//system_started = 1;
}

void send_heartbeat()
{
	MOTOR_HB_t can_msg;
	can_msg.Node_heartbeat_cmd = 0x1;
	dbc_encode_and_send_MOTOR_HB(&can_msg);
}

//uint8_t MOTOR_actual_speed;               ///< B6:0  Min: 0 Max: 75   Destination: BRIDGE,MASTER
  //  float sensed_battery_voltage
void send_feedback()
{
	MOTOR_FEEDBACK_t can_msg;
	can_msg.MOTOR_actual_speed = mps_val;
	//TBD: Add battery voltage to can message
	can_msg.sensed_battery_voltage = 0;
	dbc_encode_and_send_MOTOR_FEEDBACK(&can_msg);
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


