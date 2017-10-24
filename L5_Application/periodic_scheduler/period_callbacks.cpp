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
#include <string.h>
#include "io.hpp"
#include "periodic_callback.h"
#include "can.h"
#include "_can_dbc/generated_can.h"

bool restart_can = false;

void busoff_callback(uint32_t register_value)
{
    (void) register_value;
    restart_can = true;
}

void dataovr_callback(uint32_t register_value)
{
    (void) register_value;
}

bool dbc_app_send_can_msg(uint32_t mid, uint8_t dlc, uint8_t bytes[8])
{
    can_msg_t can_msg = { 0 };
    can_msg.msg_id                = mid;
    can_msg.frame_fields.data_len = dlc;
    memcpy(can_msg.data.bytes, bytes, dlc);

    return CAN_tx(can1, &can_msg, 0);
}


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
    CAN_init(can1, 100, 25, 25, busoff_callback, dataovr_callback);

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


/**
 * Below are your periodic functions.
 * The argument 'count' is the number of times each periodic task is called.
 */

bool start_sent = false;

void period_1Hz(uint32_t count)
{
    if(restart_can == true)
    {
        restart_can = false;
        CAN_reset_bus(can1);
        LE.toggle(1);
    }
    can_msg_t can_msg;
    BRIDGE_START_STOP_t bridge_data = {0};
    MASTER_CONTROL_t start_cmd;
    start_cmd.MASTER_CONTROL_cmd = DRIVER_HEARTBEAT_cmd_START;

    if(start_sent == false)
    {
        if(CAN_rx(can1, &can_msg, 0))
        {
            // Form the message header from the metadata of the arriving message
            dbc_msg_hdr_t can_msg_hdr;
            can_msg_hdr.dlc = can_msg.frame_fields.data_len;
            can_msg_hdr.mid = can_msg.msg_id;

            // Attempt to decode the message
            dbc_decode_BRIDGE_START_STOP(&bridge_data, can_msg.data.bytes, &can_msg_hdr);

            if(bridge_data.BRIDGE_START_STOP_cmd == 1)
            {
                LE.set(2, true);
                dbc_encode_and_send_MASTER_CONTROL(&start_cmd);
                start_sent = true;
            }
        }
    }
}

void period_10Hz(uint32_t count)
{
    can_msg_t can_msg;
    SENSOR_DATA_t sensor_data;
    MOTOR_UPDATE_t motor_update;
    motor_update.MOTOR_speed = 10;
    motor_update.MOTOR_turn_angle = 0;

    if(start_sent == true)
    {
        if(CAN_rx(can1, &can_msg, 0))
        {
            LE.toggle(3);

            // Form the message header from the metadata of the arriving message
            dbc_msg_hdr_t can_msg_hdr;
            can_msg_hdr.dlc = can_msg.frame_fields.data_len;
            can_msg_hdr.mid = can_msg.msg_id;

            dbc_decode_SENSOR_DATA(&sensor_data, can_msg.data.bytes, &can_msg_hdr);

            if(sensor_data.SENSOR_left_sensor > 1 && sensor_data.SENSOR_left_sensor < 10)
            {
                motor_update.MOTOR_turn_angle = 30;
            }
            else if(sensor_data.SENSOR_right_sensor > 1 && sensor_data.SENSOR_right_sensor < 10)
            {
                motor_update.MOTOR_turn_angle = -30;
            }
            else if(sensor_data.SENSOR_middle_sensor > 6 && sensor_data.SENSOR_middle_sensor < 15)
            {
                motor_update.MOTOR_speed = 0;
                motor_update.MOTOR_turn_angle = 0;
            }
            dbc_encode_and_send_MOTOR_UPDATE(&motor_update);
        }
    }
}

void period_100Hz(uint32_t count)
{
    //LE.toggle(3);
}

// 1Khz (1ms) is only run if Periodic Dispatcher was configured to run it at main():
// scheduler_add_task(new periodicSchedulerTask(run_1Khz = true));
void period_1000Hz(uint32_t count)
{
    //LE.toggle(4);
}
