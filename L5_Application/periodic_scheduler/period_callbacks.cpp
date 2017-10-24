/*
 *     SocialLedge.com - Copyright (C) 2013
 *
 *     This file is part of free software framework for embedded processors.
 *     You can use it and/or distribute it as long as this copyright header
 *     remains unmodified.  The code is free for personal use and requires
 *     permission to use in a commercial product.
 *
 *      THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 *      OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 *      MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 *      I SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
 *      CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 *     You can reach the author of this software at :
 *          p r e e t . w i k i @ g m a i l . c o m
 */

/**
 * @file
 * This contains the period callback functions for the periodic scheduler
 *
 * @warning
 * These callbacks should be used for hard real-time system, and the priority of these
 * tasks are above everything else in the system (above the PRIORITY_CRITICAL).
 * The period functions SHOULD NEVER block and SHOULD NEVER run over their time slot.
 * For example, the 1000Hz take slot runs periodically every 1ms, and whatever you
 * do must be completed within 1ms.  Running over the time slot will reset the system.
 */

#include <stdint.h>
#include "io.hpp"
#include "periodic_callback.h"
#include "eint.h"
#include "gpio.hpp"
#include <stdio.h>
#include "uart3.hpp"
#include "lpc_pwm.hpp"
#include "can.h"
#include "motor_controller/motor.hpp"

/// This is the stack size used for each of the period tasks (1Hz, 10Hz, 100Hz, and 1000Hz)
const uint32_t PERIOD_TASKS_STACK_SIZE_BYTES = (512 * 4);

/**
 * This is the stack size of the dispatcher task that triggers the period tasks to run.
 * Minimum 1500 bytes are needed in order to write a debug file if the period tasks overrun.
 * This stack size is also used while calling the period_init() and period_reg_tlm(), and if you use
 * printf inside these functions, you need about 1500 bytes minimum
 */
const uint32_t PERIOD_MONITOR_TASK_STACK_SIZE_BYTES = (512 * 3);

/// Called once before the RTOS is started, this is a good place to initialize things once
bool period_init(void)
{
	bool rc;
	MOTOR = get_motor_pwm(PWM::pwm1);
	SERVO = get_servo_pwm(PWM::pwm2);
	stop_car();
	system_started = 0;

	//Start interrupt to count wheel rotations
	eint3_enable_port2(0, eint_falling_edge, rpm_intr_hdlr);

	//Enable can1 to rx/tx messages
	rc = CAN_init(can1, 100, 20, 20, NULL, NULL);
	printf("CAN init rc %d\n", rc);
	CAN_bypass_filter_accept_all_msgs();
	CAN_reset_bus(can1);

    return true; // Must return true upon success
}

/// Register any telemetry variables
bool period_reg_tlm(void)
{
    // Make sure "SYS_CFG_ENABLE_TLM" is enabled at sys_config.h to use Telemetry
    return true; // Must return true upon success
}

void send_simple_can(void)
{
	can_msg_t can_tx_msg;
	bool rc;
	can_tx_msg.msg_id = 0x122;
	can_tx_msg.frame_fields.is_29bit = 0;
	can_tx_msg.frame_fields.data_len = 1;       // Send 8 bytes

	if (SW.getSwitch(1))
	{
		can_tx_msg.data.bytes[0] = 0xAA;
		//If switch is pressed, send 0xAA
		rc = CAN_tx(can1, &can_tx_msg, portMAX_DELAY);
		printf("sent message, rc, %d\n", rc);
	} else {
		can_tx_msg.data.bytes[0] = 0x00;
		//If switch not pressed, send 0x00
		CAN_tx(can1, &can_tx_msg, portMAX_DELAY);
	}
}

void rx_simple_can(void)
{
	bool rc;
	can_msg_t can_rx_msg;
	rc = CAN_rx(can1, &can_rx_msg, 0x50);
	if (can_rx_msg.data.bytes[0] == 0xAA)
	{
		printf("Received data 0xAA on CAN\n");
		LE.on(1);
	} else if (can_rx_msg.data.bytes[0] == 0xAA) {
		printf("received data %x\n", can_rx_msg.data.bytes[0]);
	} else {
		LE.off(1);
	}
}

/**
 * Below are your periodic functions.
 * The argument 'count' is the number of times each periodic task is called.
 */
float duty_cycle=15;
void period_1Hz(uint32_t count)
{
	//If CAN bus turns off, re-enable it
	if (CAN_is_bus_off(can1))
	{
		printf("Can bus is off\n");
		CAN_reset_bus(can1);
	}

	//First, we receive MASTER CMD to start system
	if (!system_started)
	{
		recv_system_start();
	} else {
		//After we receive this, respond with heartbeat
		//1 HZ => motor sends heartbeat
		send_heartbeat();
	}


	//stop_car();
//	if (count == 5)
//	{
//		printf("go backward\n");
//		//set_speed(-25);
//		set_angle(-8);
//	}
//	if (count == 10)
//	{
//		printf("reset\n");
//		stop_car();
//	}
//	if (count == 15)
//	{
//		printf("go foward\n");
//		//set_speed(15);
//		set_angle(8);
//	}
//	if (count == 20)
//	{
//		stop_car();
//	}
//	if (count == 25)
//	{
//		set_speed(-15);
//	}
//	if (count == 30)
//	{
//		stop_car();
//	}

}

void period_10Hz(uint32_t count)
{
	//Send current motor/angle
	//get_rpm_val();

	//Decode & update motor speed/angle
	update_speed_and_angle();
}

void period_100Hz(uint32_t count)
{
	//Check CAN RX for change in motor/angle

}

// 1Khz (1ms) is only run if Periodic Dispatcher was configured to run it at main():
// scheduler_add_task(new periodicSchedulerTask(run_1Khz = true));
void period_1000Hz(uint32_t count)
{
    LE.toggle(4);
}
