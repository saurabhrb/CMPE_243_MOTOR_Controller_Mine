/*
 * motor.hpp
 *
 *  Created on: Oct 7, 2017
 *      Author: jskow
 */
#include "lpc_pwm.hpp"
#include "singleton_template.hpp"  // Singleton Template
#include "_can_dbc/generated_can.h"
#include "can.h"
#include "string.h"
#include "io.hpp"
#include "lpc_sys.h"
#include "LPC17xx.h"
#include "sys_config.h"
#include "stdio.h"

typedef enum {
    FWD = 1,
    REV = -1,
} Speed_dir;

class Motor : public SingletonTemplate<Motor>
{
    public:
        bool init();
        void get_can_vals(); //to update curr_can_speed, curr_can_angle, prev_can_speed, prev_can_angle
        void set_speed(); //convert speed to pwm, and handle (curr_mps_speed != 0 && (prev_can_speed > 0 && curr_can_speed < 0))
        void set_angle(); //convert angle to pwm
        void check_real_speed_update(); //to check if curr_mps_speed == curr_can_speed, if not increase prev_speed_val
        bool speed_attained();
        void stop_car();
        void motor_periodic(); //to be called in periodic function, which alls appropriate functions
        float curr_rps_cnt; //current pedometer count coming from interrupt
        void terminal_update(float sp,float an);
        bool system_started;
    private:
        Motor();  ///< Private constructor of this Singleton class
        friend class SingletonTemplate<Motor>;  ///< Friend class used for Singleton Template
        Speed_dir real_speed_dir;
        PWM * MOTOR;
        PWM * SERVO;
        float curr_can_speed; //current received speed from can message
        float curr_can_angle; //current received angle from can message
        float prev_can_speed; //last received speed from can message
        float prev_can_angle; //last received speed from can message
        float prev_speed_val; //last used speed to make curr_mps_speed close to can_speed
        float curr_mps_speed; //current real speed
        float prev_rps_cnt; //previously read pedometer count

};

//interrupt handler callback
void rps_cnt_hdlr(); //to update prev_rps_cnt and curr_rps_cnt;

//Relevant CAN helper functions for motor controller
void recv_system_start();
void send_heartbeat();

#ifndef L5_APPLICATION_MOTOR_CONTROLLER_MOTOR_HPP_
#define L5_APPLICATION_MOTOR_CONTROLLER_MOTOR_HPP_


#endif /* L5_APPLICATION_MOTOR_CONTROLLER_MOTOR_HPP_ */
