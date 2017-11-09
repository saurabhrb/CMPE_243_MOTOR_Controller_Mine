/*
 * 	This file contains motor controller functions for the Traxxas RC car.
 *  Control is provided for the motor driver and servo motor (left/right).
 *
 * TBD: Go forward/reverse by distance
 *      Speed feedback test with speed_attained function
 */

#include "motor.hpp"

#define speed_margin 0.05
#define speed_step 0.25 //speed increment step for check_speed

#define DIA_m 0.2 //0.05588 //in meters = 2.2 inches

#define DIA_m_mul_PI (DIA_m * 3.14159265359)
#define CIRCUMFERENCE 0.175552197 * (DIA_m/0.05588) //DIA_m_mul_PI metres

#define Traxxas_Max_Speed 33.528 //mps
#define duty_factor (5.0/Traxxas_Max_Speed) //required pwm for 1ms => 15.0 +/- (1ms * duty_factor)
#define neutral_pwm 15.0

Motor::Motor()
{
    use_prev_speed = false;
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


//Bring the car to a use_prev_speed at neutral steering position, reset all Motor class values
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
    //this->check_real_speed_update();
}

void Motor::terminal_update(char a,float var)
{
    switch(a)
    {
        case '1':
            if(var >= 10.0)
                curr_can_speed = 10.0;
            else if(var <= -10.0)
                curr_can_speed = -10.0;
            else
                curr_can_speed = var;
            break;
        case '2':
            if(var >= 30.0)
                curr_can_angle = 30.0;
            else if(var <= -30.0)
                curr_can_angle = -30.0;
            else
                curr_can_angle = var;
            break;
        default:
            break;
    }
}

void Motor::get_can_vals() //to update curr_can_speed, curr_can_angle, prev_can_speed, prev_can_angle
{
    can_msg_t can_msg;
    while (CAN_rx(can1, &can_msg, 0))
    {
        // Form the message header from the metadata of the arriving message
        dbc_msg_hdr_t can_msg_hdr;
        can_msg_hdr.dlc = can_msg.frame_fields.data_len;
        can_msg_hdr.mid = can_msg.msg_id;

        if(can_msg_hdr.mid == MASTER_CONTROL_HDR.mid)
        {
            MASTER_CONTROL_t master_can_msg;
            // Attempt to decode the message (brute force, but should use switch/case with MID)
            if (dbc_decode_MASTER_CONTROL(&master_can_msg, can_msg.data.bytes, &can_msg_hdr))
            {
                if (master_can_msg.MASTER_CONTROL_cmd == DRIVER_HEARTBEAT_cmd_START)
                {
                    //////printf("recv start\n");
                    Motor::getInstance().system_started = 1;
                    //Motor::getInstance().motor_periodic();
                }
                else if (master_can_msg.MASTER_CONTROL_cmd == DRIVER_HEARTBEAT_cmd_RESET)
                {
                    Motor::getInstance().stop_car();
                    //////printf("recv start\n");
                    Motor::getInstance().system_started = 0;
                }
                LE.on(2);
            }

        }
        else if(can_msg_hdr.mid == MOTOR_UPDATE_HDR.mid)
        {
            MOTOR_UPDATE_t motor_can_msg;
            // Attempt to decode the message (brute force, but should use switch/case with MID)
            if (dbc_decode_MOTOR_UPDATE(&motor_can_msg, can_msg.data.bytes, &can_msg_hdr))
            {
                 curr_can_speed = ((float)motor_can_msg.MOTOR_speed);
                 if(curr_can_speed < 0)
                     curr_can_speed = 0;
                 curr_can_angle = ((float)motor_can_msg.MOTOR_turn_angle);
                 LE.on(2);
                 LD.setNumber((int)curr_can_speed);
            }
        }
    }

}

void Motor::set_speed() //convert speed to pwm, and handle (curr_mps_speed != 0 && (prev_can_speed > 0 && curr_can_speed < 0))
{


    if(curr_mps_speed > 0 &&  curr_can_speed < 0)
        {
                if(curr_mps_speed != 0)
                {
                    use_prev_speed = 1;
                    prev_can_speed = 0;
                    this->check_real_speed_update();
                    prev_can_speed = curr_can_speed;
                    return;
                }
        }

    if(prev_can_speed == curr_can_speed)
    {
        this->check_real_speed_update();
    }
    else if(prev_can_speed != curr_can_speed)
    {
        ////printf("prev_can_speed = %f -> curr_can_speed = %f, prev_speed_val = %f", prev_can_speed, curr_can_speed, prev_speed_val);
        //if(curr_can_speed < 0 && )
        //prev_speed_val = curr_can_speed;
        if(abs(prev_speed_val*duty_factor) <= 5.0)
        {
            ////printf("\n pwm = %f",neutral_pwm + (prev_speed_val*duty_factor));
            //go more forward/reverse
            MOTOR->set(neutral_pwm + (prev_speed_val*duty_factor));
        }
        else if(prev_speed_val*duty_factor < 0)
        {
            ////printf("\n pwm = %f",neutral_pwm + (prev_speed_val*duty_factor));
            //runnning at max speed possible in reverse
            MOTOR->set(neutral_pwm - (5.0));
        }
        else if(prev_speed_val*duty_factor > 0)
        {
            ////printf("\n pwm = %f",neutral_pwm + (prev_speed_val*duty_factor));
            //runnning at max speed possible in forward
            MOTOR->set(neutral_pwm + (5.0));
        }

        prev_can_speed = curr_can_speed;
        if(curr_can_speed < 0)
            real_speed_dir = REV;
        else
            real_speed_dir = FWD;
    }
}

void Motor::set_angle() //convert angle to pwm
{
    if(prev_can_angle != curr_can_angle)
    {
        ////printf("prev_can_angle = %f -> curr_can_angle = %f", prev_can_angle, curr_can_angle);
        prev_can_angle = curr_can_angle;
    //max angle is 30
    if(abs(curr_can_angle*duty_factor) <= 5.0)
        SERVO->set(neutral_pwm + (curr_can_angle * (5.0/30.0)));
    else
        SERVO->set(neutral_pwm + (5.0));
    }
}

void Motor::check_real_speed_update() //to check if curr_mps_speed == curr_can_speed, if not increase prev_speed_val
{

    int diff_cnt = curr_rps_cnt;
    curr_rps_cnt = 0;

    if((diff_cnt)!=0)
        ;//printf("\n diff_cnt = %d, prev_speed_val = %f\n",diff_cnt, prev_speed_val);
    else
        curr_mps_speed = 0.0;

    if(curr_mps_speed != 0.0)
    {
        printf("\ncurr_mps_speed = %f\n", curr_mps_speed);
    }
    else
    {
        if(use_prev_speed == true)
        {
            prev_speed_val = prev_can_speed;
            use_prev_speed = false;
        }
        else
        {
            prev_speed_val = curr_can_speed;
        }
    }


    //curr_mps_speed = CIRCUMFERENCE*(diff_cnt)/0.1;
    curr_mps_speed = real_speed_dir * (3.14*0.04)*(diff_cnt)/0.1;
    LD.setNumber(abs(curr_mps_speed));

    if(curr_can_speed == 0.0)
        {
            this->stop_car();
            return;
        }

    if(this->speed_attained() == false)
    {
        float delta = curr_mps_speed - (curr_can_speed);
        printf("\ndelta = %f\n",delta);

        if(!(abs(delta) > 0 && abs(delta) < speed_margin))
          {
                //for quicker speed up/down
                //if(abs(delta)>=speed_step*4)
                //    prev_speed_val+=speed_step*(abs(delta)/speed_step);
                //else
                    if(curr_can_speed < curr_mps_speed)
                        prev_speed_val-=speed_step;
                    else if(curr_can_speed > curr_mps_speed)
                        prev_speed_val+=speed_step;
                        //this->set_speed();

                    printf("prev_speed_val = %f\n",prev_speed_val);
                    if(abs(prev_speed_val*duty_factor) <= 5.0)
                                                {
                                                    ////printf("\n pwm = %f",neutral_pwm + (prev_speed_val*duty_factor));
                                                    //go more forward/reverse
                                                    MOTOR->set(neutral_pwm + (prev_speed_val*duty_factor));
                                                }
                                                else if(prev_speed_val*duty_factor < 0)
                                                {
                                                    ////printf("\n pwm = %f",neutral_pwm + (prev_speed_val*duty_factor));
                                                    //runnning at max speed possible in reverse
                                                    MOTOR->set(neutral_pwm - (5.0));
                                                }
                                                else if(prev_speed_val*duty_factor > 0)
                                                {
                                                    ////printf("\n pwm = %f",neutral_pwm + (prev_speed_val*duty_factor));
                                                    //runnning at max speed possible in forward
                                                    MOTOR->set(neutral_pwm + (5.0));
                                                }

            }
    }
    prev_rps_cnt = curr_rps_cnt;
}

bool Motor::speed_attained()
{
    if(real_speed_dir == REV && prev_can_speed ==0)
        {
            prev_can_speed = curr_can_speed;
            return true;
        }
    if(abs(curr_can_speed - curr_mps_speed) >=0 && abs(curr_can_speed - curr_mps_speed) <= speed_margin)
        return true;
    else
        return false;
}

float Motor::get_curr_rps_speed()
{
    return curr_mps_speed;
}


/////////HELPER functions////////

void rps_cnt_hdlr() //to update prev_rps_cnt and curr_rps_cnt;
{
    Motor::getInstance().curr_rps_cnt++;
}

void send_heartbeat()
{
    MOTOR_HB_t can_msg;
    can_msg.MOTOR_heartbeat = 0x1;
    dbc_encode_and_send_MOTOR_HB(&can_msg);
    LE.on(3);
}

void send_feedback()
{
    MOTOR_FEEDBACK_t can_msg;
    can_msg.MOTOR_actual_speed = Motor::getInstance().get_curr_rps_speed();
    can_msg.sensed_battery_voltage = 0;
    dbc_encode_and_send_MOTOR_FEEDBACK(&can_msg);
    LE.on(4);
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
        if(can_msg_hdr.mid == MASTER_CONTROL_HDR.mid)
                {
                    MASTER_CONTROL_t master_can_msg;
                    // Attempt to decode the message (brute force, but should use switch/case with MID)
                    if (dbc_decode_MASTER_CONTROL(&master_can_msg, can_msg.data.bytes, &can_msg_hdr))
                    {
                        if (master_can_msg.MASTER_CONTROL_cmd == DRIVER_HEARTBEAT_cmd_START)
                        {
                            //////printf("recv start\n");
                            Motor::getInstance().system_started = 1;
                            //Motor::getInstance().motor_periodic();
                        }
                        else if (master_can_msg.MASTER_CONTROL_cmd == DRIVER_HEARTBEAT_cmd_RESET)
                        {
                            Motor::getInstance().stop_car();
                            //////printf("recv start\n");
                            Motor::getInstance().system_started = 0;
                        }
                        LE.on(2);
                    }

                }
    }
    // Service the MIA counters
    // successful decoding resets the MIA counter, otherwise it will increment to
    // its MIA value and upon the MIA trigger, it will get replaced by your MIA struct
    //rc = dbc_handle_mia_LAB_TEST(&master_can_msg, 1000);  // 1000ms due to 1Hz
    //system_started = 1;
}
