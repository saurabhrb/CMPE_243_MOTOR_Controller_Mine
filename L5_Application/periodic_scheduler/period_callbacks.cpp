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

/*
LE(1); //on if system started
LE(2); //on if any can_msg is successfully decoded
LE(3); //on if sending Motor_heartbeat over can
LE(4); //on if sending Motor_feedback over can
*/


/// This is the stack size used for each of the period tasks (1Hz, 10Hz, 100Hz, and 1000Hz)
const uint32_t PERIOD_TASKS_STACK_SIZE_BYTES = (512 * 4);

/**
 * This is the stack size of the dispatcher task that triggers the period tasks to run.
 * Minimum 1500 bytes are needed in order to write a debug file if the period tasks overrun.
 * This stack size is also used while calling the period_init() and period_reg_tlm(), and if you use
 * printf inside these functions, you need about 1500 bytes minimum
 */
const uint32_t PERIOD_MONITOR_TASK_STACK_SIZE_BYTES = (512 * 3);

//flag to see first periodic function call or not
bool first_time;

// Called once before the RTOS is started, this is a good place to initialize things once
bool period_init(void)
{
	bool rc;

	first_time = 1;

	Motor::getInstance().init(); //reset motors of the car with all values set to 0

	//Start interrupt to count wheel rotations
	eint3_enable_port2(0, eint_falling_edge, rps_cnt_hdlr);

	//Enable can1 to rx/tx messages
	rc = CAN_init(can1, 100, 20, 20, NULL, NULL);

	//printf("CAN init rc %d\n", rc);
	CAN_bypass_filter_accept_all_msgs();
	CAN_reset_bus(can1);

    return true; // Must return true upon success
}

// Register any telemetry variables
bool period_reg_tlm(void)
{
    // Make sure "SYS_CFG_ENABLE_TLM" is enabled at sys_config.h to use Telemetry
    return true; // Must return true upon success
}

/**
 * Below are your periodic functions.
 * The argument 'count' is the number of times each periodic task is called.
 */
void period_1Hz(uint32_t count)
{
    return;
	//If CAN bus turns off, re-enable it
	if (CAN_is_bus_off(can1))
	{
		//printf("Can bus is off\n");
		CAN_reset_bus(can1);
	}

	if(first_time)
	{
	    recv_system_start();
	    first_time = 0;
	}

	if(Motor::getInstance().system_started)
	{
	    send_heartbeat();
	    LE.on(1);
	}
}

void period_10Hz(uint32_t count)
{
	 if (Motor::getInstance().system_started)
    {
        Motor::getInstance().motor_periodic();
        send_feedback();
        LE.on(1);
    }
    else
    {
        Motor::getInstance().init(); //reset car
    }

	LE.off(1);
    LE.off(2);
    LE.off(3);
    LE.off(4);

}

void period_100Hz(uint32_t count)
{
    /*if (Motor::getInstance().system_started)
    {
        Motor::getInstance().motor_periodic();
        send_feedback();
        LE.on(1);
    }
    else
    {
        Motor::getInstance().init(); //reset car
    }

    LE.off(1);
    LE.off(2);
    LE.off(3);
    LE.off(4);*/

}

// 1Khz (1ms) is only run if Periodic Dispatcher was configured to run it at main():
// scheduler_add_task(new periodicSchedulerTask(run_1Khz = true));
void period_1000Hz(uint32_t count)
{
    //LE.toggle(4);
}
