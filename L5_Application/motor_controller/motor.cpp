/*
 * 	This file contains motor controller functions for the Traxxas RC car.
 *  Control is provided for the motor driver and servo motor (left/right).
 *
 * TBD: Go forward/reverse by distance
 * 		Take in angle required for turning left/right
 * 		Use singleton instance to access PWM
 */

#include "motor.hpp"

#define speed_margin 0.5
#define speed_step 0.25 //speed increment step for check_speed

#define DIA_m 0.05588 //in meters = 2.2 inches

#define DIA_m_mul_PI (DIA_m * 3.14159265359)
#define CIRCUMFERENCE 0.175552197 * (DIA_m/0.0558) //DIA_m_mul_PI metres

#define Traxxas_Max_Speed 33.528 //mps
#define duty_factor (5.0/Traxxas_Max_Speed) //required pwm for 1ms => 15.0 +/- (1ms * duty_factor)
#define neutral_pwm 15.0

#define MOTOR_UPDATE_CMD_MSG_ID 300
#define MOTOR_CONTROL_CMD_MSG_ID 100


Motor::Motor()
{
    system_started = 0;
    static PWM motor(PWM::pwm2, 100);
    motor.set(15.0);
    MOTOR = &motor;
    static PWM servo(PWM::pwm3, 100);
    servo.set(15.0);
    SERVO = &servo;
    real_speed_dir = FWD;
    curr_can_speed = 0; //current received speed from can message
    curr_can_angle = 0; //current received angle from can message
    prev_can_speed = 0; //last received speed from can message
    prev_can_angle = 0; //last received speed from can message
    prev_speed_val = 0; //last used speed to make curr_mps_speed close to can_speed
    curr_mps_speed = 0; //current real speed
    prev_rps_cnt = 0; //previously read pedometer count
    curr_rps_cnt = 0; //current pedometer count coming from interrupt
}

bool Motor::init()
{
    MOTOR->set(neutral_pwm);
    SERVO->set(neutral_pwm);
    real_speed_dir = FWD;
    curr_can_speed = 0; //current received speed from can message
    curr_can_angle = 0; //current received angle from can message
    prev_can_speed = 0; //last received speed from can message
    prev_can_angle = 0; //last received speed from can message
    prev_speed_val = 0; //last used speed to make curr_mps_speed close to can_speed
    curr_mps_speed = 0; //current real speed
    prev_rps_cnt = 0; //previously read pedometer count
    curr_rps_cnt = 0; //current pedometer count coming from interrupt
    return true;
}

//Bring the car to a halt at neutral steering position, reset all Motor class values
void Motor::stop_car()
{
        MOTOR->set(neutral_pwm);
        SERVO->set(neutral_pwm);
        real_speed_dir = FWD;
        prev_can_speed = 0; //last received speed from can message
        prev_can_angle = 0; //last received speed from can message
        prev_speed_val = 0; //last used speed to make curr_mps_speed close to can_speed
        prev_rps_cnt = 0; //previously read pedometer count
        curr_rps_cnt = 0; //current pedometer count coming from interrupt
}

void Motor::motor_periodic()
{
    this->get_can_vals();
    this->set_speed();
    this->set_angle();
    this->check_real_speed_update();
}

void Motor::terminal_update(float sp,float an)
{
    curr_can_speed = ((sp>=10)?10:((sp<=-10)?-10:sp));
    curr_can_angle = ((an>=30)?30:((an<=-30)?-30:an));
}

void Motor::get_can_vals() //to update curr_can_speed, curr_can_angle, prev_can_speed, prev_can_angle
{
    bool rc = 0;

    can_msg_t can_msg;
    while (CAN_rx(can1, &can_msg, 0))
    {
        // Form the message header from the metadata of the arriving message
        dbc_msg_hdr_t can_msg_hdr;
        can_msg_hdr.dlc = can_msg.frame_fields.data_len;
        can_msg_hdr.mid = can_msg.msg_id;

        if(can_msg_hdr.mid == MOTOR_CONTROL_CMD_MSG_ID)
        {
            MASTER_CONTROL_t master_can_msg;
            // Attempt to decode the message (brute force, but should use switch/case with MID)
            rc = dbc_decode_MASTER_CONTROL(&master_can_msg, can_msg.data.bytes, &can_msg_hdr);
            if (rc == true)
            {
                if (can_msg.data.bytes[0] == MASTER_cmd_START)
                {
                    LE.on(1);
                    LD.setNumber(1);
                    //printf("recv start\n");
                    system_started = 1;
                    //Motor::getInstance().motor_periodic();
                }
                else if (can_msg.data.bytes[0] == MASTER_cmd_STOP)
                {
                    Motor::getInstance().stop_car();
                    LE.off(1);
                    LD.setNumber(0);
                    //printf("recv start\n");
                    system_started = 0;
                }

            }

        }
        else if(can_msg_hdr.mid == MOTOR_UPDATE_CMD_MSG_ID)
        {
            MOTOR_UPDATE_t motor_can_msg;
            // Attempt to decode the message (brute force, but should use switch/case with MID)
            rc = dbc_decode_MOTOR_UPDATE(&motor_can_msg, can_msg.data.bytes, &can_msg_hdr);
            if (rc == true)
            {
                 curr_can_speed = ((float)motor_can_msg.MOTOR_speed);
                 curr_can_angle = ((float)motor_can_msg.MOTOR_turn_angle);
            }
        }
    }
}

void Motor::set_speed() //convert speed to pwm, and handle (curr_mps_speed != 0 && (prev_can_speed > 0 && curr_can_speed < 0))
{
    if(curr_mps_speed != 0 && (prev_can_speed > 0 && curr_can_speed < 0))
    {
        //MOTOR->set(neutral_pwm + (0*duty_factor));
        //prev_can_speed =0;
        this->stop_car();

    }
    else if(prev_can_speed == curr_can_speed)
    {
        this->check_real_speed_update();
    }
    else
    {
        if(prev_speed_val ==0)
               prev_speed_val = curr_can_speed;

        if(prev_speed_val*duty_factor <= 5.0)
            MOTOR->set(neutral_pwm + (prev_speed_val*duty_factor));
        else
            MOTOR->set(neutral_pwm + (5.0));
        prev_can_speed = curr_can_speed;
    }
}

void Motor::set_angle() //convert angle to pwm
{
    //max angle is 30
    if(curr_can_angle*duty_factor <= 5.0)
        SERVO->set(neutral_pwm + (curr_can_angle * (5.0/30.0)));
    else
        SERVO->set(neutral_pwm + (5.0));

    prev_can_angle = curr_can_angle;
}

void Motor::check_real_speed_update() //to check if curr_mps_speed == curr_can_speed, if not increase prev_speed_val
{
    if(curr_rps_cnt - prev_rps_cnt > 1000000000)
        return;
    curr_mps_speed = CIRCUMFERENCE*(curr_rps_cnt - prev_rps_cnt)/0.1;
    LD.setNumber(abs(curr_mps_speed));

    if(curr_can_speed == 0.0)
        {
            this->stop_car();
            return;
        }

    if(!this->speed_attained())
    {
        float delta = curr_mps_speed - curr_can_speed;
        if(!(abs(delta) > 0 && abs(delta) < speed_margin))
            {
                //for quicker speed up/down
                //if(abs(delta)>=speed_step*4)
                //    prev_speed_val+=speed_step*(abs(delta)/speed_step);
                //else
                    prev_speed_val+=speed_step;
                    this->set_speed();
            }
    }
    prev_rps_cnt = curr_rps_cnt;
}

bool Motor::speed_attained()
{
    if(abs(curr_can_speed - curr_mps_speed) >=0 && abs(curr_can_speed - curr_mps_speed) <= speed_margin)
        return true;
    else
        return false;
}

void rps_cnt_hdlr() //to update prev_rps_cnt and curr_rps_cnt;
{
    Motor::getInstance().curr_rps_cnt++;
}

void send_heartbeat()
{
    MOTOR_HB_t can_msg;
    can_msg.Node_heartbeat_cmd = 0x1;
    dbc_encode_and_send_MOTOR_HB(&can_msg);
}

bool dbc_app_send_can_msg(uint32_t mid, uint8_t dlc, uint8_t bytes[8])
{
    can_msg_t can_msg = { 0 };
    can_msg.msg_id                = mid;
    can_msg.frame_fields.data_len = dlc;
    memcpy(can_msg.data.bytes, bytes, dlc);

    return CAN_tx(can1, &can_msg, 0);
}

//Scan for start/stop command from master node
void recv_system_start()
{

    MASTER_CONTROL_t master_can_msg;
    can_msg_t can_msg;
    bool rc;
    while (CAN_rx(can1, &can_msg, 0))
    {
        // Form the message header from the metadata of the arriving message
        dbc_msg_hdr_t can_msg_hdr;
        can_msg_hdr.dlc = can_msg.frame_fields.data_len;
        can_msg_hdr.mid = can_msg.msg_id;

        // Attempt to decode the message (brute force, but should use switch/case with MID)
                    rc = dbc_decode_MASTER_CONTROL(&master_can_msg, can_msg.data.bytes, &can_msg_hdr);
                    if (rc == true)
                    {
                        if (can_msg.data.bytes[0] == MASTER_cmd_START)
                        {
                            LE.on(1);
                            LD.setNumber(1);
                            //printf("recv start\n");
                            Motor::getInstance().system_started = 1;
                            //Motor::getInstance().motor_periodic();
                        }
                        else if (can_msg.data.bytes[0] == MASTER_cmd_STOP)
                        {
                            Motor::getInstance().stop_car();
                            LE.off(1);
                            LD.setNumber(0);
                            //printf("recv start\n");
                            Motor::getInstance().system_started = 0;
                        }

                    }
    }
    // Service the MIA counters
    // successful decoding resets the MIA counter, otherwise it will increment to
    // its MIA value and upon the MIA trigger, it will get replaced by your MIA struct
    //rc = dbc_handle_mia_LAB_TEST(&master_can_msg, 1000);  // 1000ms due to 1Hz
    //system_started = 1;
}
