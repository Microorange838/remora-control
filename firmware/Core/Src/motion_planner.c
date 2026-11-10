/*************************************************************
 * @file motion_planner.c
 * @author MicroOrange
 * @brief 运动规划器层 反馈与速度
 * @date 2022.2.23
 *************************************************************/

#include <stdlib.h>
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "string.h"
#include "stdio.h"
#include "pid.h"
#include "motor.h"
#include "motor.h"
#include "fish_controller.h"
#include "IMU.h"
#include "cpg_controller.h"
#include "speed_convert_layer_v2.h"
#include "math.h"
#include "para_service.h"
#include "motion_planner.h"
#include "atk_ms53l1m.h"
#include "network_app.h"
#include "log.h"
#include "time_measure.h"

uint8_t motion_mode = MOTION_MODE_REMOTE;

pid_type pid_rotate_yaw;
pid_type pid_rotate_pitch;
pid_type pid_rotate_roll;

double test1_freq = 1;

double motion_threedimrotate_target_y=0;
double motion_threedimrotate_target_p=0;
double motion_threedimrotate_target_r=0;

double motion_test2_vf = 0;
double motion_test2_vd = 0;
double motion_test2_vr = 0;
double motion_test2_vp = 0;
double motion_test2_vy = 0;

double channel_value[10];


void Motion_Init(){
    PID_init(&pid_rotate_yaw, PID_POSITION, 0.00, 0, 0, 2, 3);
    PID_init(&pid_rotate_pitch, PID_POSITION, -3, 0, 0, 50, 10);
    PID_init(&pid_rotate_roll, PID_POSITION, 0.00, 0, 0, 20, 3);
}



void Motion_ParaThreeDimRotate(int para_index, double value){
    switch (para_index) {
        case PARA_MOTION_THREEDIMROTATE_PID_R_KP:
            pid_rotate_roll.Kp = value;
            break;
        case PARA_MOTION_THREEDIMROTATE_PID_R_KI:
            pid_rotate_roll.Ki = value;
            break;
        case PARA_MOTION_THREEDIMROTATE_PID_R_KD:
            pid_rotate_roll.Kd = value;
            break;
        case PARA_MOTION_THREEDIMROTATE_PID_P_KP:
            pid_rotate_pitch.Kp = value;
            break;
        case PARA_MOTION_THREEDIMROTATE_PID_P_KI:
            pid_rotate_pitch.Ki = value;
            break;
        case PARA_MOTION_THREEDIMROTATE_PID_P_KD:
            pid_rotate_pitch.Kd = value;
            break;
        case PARA_MOTION_THREEDIMROTATE_PID_Y_KP:
            pid_rotate_yaw.Kp = value;
            break;
        case PARA_MOTION_THREEDIMROTATE_PID_Y_KI:
            pid_rotate_yaw.Ki = value;
            break;
        case PARA_MOTION_THREEDIMROTATE_PID_Y_KD:
            pid_rotate_yaw.Kd = value;
            break;
        case PARA_MOTION_THREEDIMROTATE_R:
            motion_threedimrotate_target_r = value;
            break;
        case PARA_MOTION_THREEDIMROTATE_P:
            motion_threedimrotate_target_p = value;
            break;
        case PARA_MOTION_THREEDIMROTATE_Y:
            motion_threedimrotate_target_y = value;
            break;

        default:
            break;
    }
}

void Motion_ParaTest2(int para_index, double value){
    switch (para_index) {
        case PARA_MOTION_MODE_TEST2_Vf:
            motion_test2_vf = value;
            break;
        case PARA_MOTION_MODE_TEST2_Vd:
            motion_test2_vd = value;
            break;
        case PARA_MOTION_MODE_TEST2_Vr:
            motion_test2_vr = value;
            break;
        case PARA_MOTION_MODE_TEST2_Vp:
            motion_test2_vp = value;
            break;
        case PARA_MOTION_MODE_TEST2_Vy:
            motion_test2_vy = value;
            break;
        case PARA_REMOTE_CHANNEL_1:
            channel_value[0] = value;
            break;
        case PARA_REMOTE_CHANNEL_2:
            channel_value[1] = value;
            break;
        case PARA_REMOTE_CHANNEL_3:
            channel_value[2] = value;
            break;
        case PARA_REMOTE_CHANNEL_4:
            channel_value[3] = value;
            break;
        case PARA_REMOTE_CHANNEL_5:
            channel_value[4] = value;
            break;
        case PARA_REMOTE_CHANNEL_6:
            channel_value[5] = value;
            break;
        case PARA_REMOTE_CHANNEL_7:
            channel_value[6] = value;
            break;
        case PARA_REMOTE_CHANNEL_8:
            channel_value[7] = value;
            break;
        default:
            break;
    }
}

void Motion_Remote_Control(fish_t *remora){
    double neck = channel_value[3];
    double pec = channel_value[2];
    pec = pec*4;
    if (pec>100.0)
        pec = 100.0;
    double front = channel_value[0];
    double yaw = channel_value[0];
    double disc_control = channel_value[5];
    int neck_mode = lround(channel_value[4]);
    int pec_front_mode = 0;

    double f_pec = fabs(pec);
    double pec_pitch_amp = 0.9 ;
    double pec_pitch_freq = 0.04 ;
    double pec_roll_amp = 0.8;
    double pec_roll_freq = 0.04;
    double tail_freq = 0.02;
    double tail_offset = 0.04;
    double plus_count=1 ;

    if (fabs(front) < 5){
        remora->cpg_tail1.amplitude = 0;
        remora->cpg_tail2.amplitude = 0;
        remora->cpg_tail1.offset = 0;
        remora->cpg_tail2.offset = 0;

    } else{
        remora->cpg_tail1.amplitude = 25;
        remora->cpg_tail2.amplitude = 30;
        remora->cpg_tail1.offset = yaw * tail_offset;
        remora->cpg_tail2.offset = yaw * tail_offset*1.5;
        //  remora->cpg_tail1.offset = yaw * 0.3;
//        CPG_SetLinkPara(&remora->cpg_tail1, &remora->cpg_tail2, 1, front_bias/100*PI);
//        CPG_SetLinkPara(&remora->cpg_tail2, &remora->cpg_tail1, 1, -front_bias/100*PI);
        //remora->cpg_tail2.offset = yaw * 0.6;
    }
//    remora->cpg_tail1.frequency = front*tail_freq;
//    remora->cpg_tail2.frequency = front*tail_freq;
    remora->cpg_tail1.frequency = 2.5;
    remora->cpg_tail2.frequency = 2;

    if(fabs(disc_control)<5){
        remora->bumper_disc.mode = BUMPER_MODE_POS;
        remora->bumper_disc.set = 0;
    }
    else {
        remora->bumper_disc.mode = BUMPER_MODE_POS;
        remora->bumper_disc.set = disc_control*1.75   ;
    }


    if (fabs(pec) > 5) {
        if (pec > 0) {
            remora->cpg_left_pectrol.offset = -70;
            remora->cpg_right_pectrol.offset = -70;
        } else {
            remora->cpg_left_pectrol.offset = 70;
            remora->cpg_right_pectrol.offset = 70;
        }
        remora->cpg_left_pectrol.amplitude = f_pec * pec_roll_amp * plus_count;
        remora->cpg_right_pectrol.amplitude = f_pec * pec_roll_amp;
        remora->cpg_left_pectrol.frequency = f_pec * pec_roll_freq;
        remora->cpg_right_pectrol.frequency = f_pec * pec_roll_freq;
    } else {
        remora->cpg_left_pectrol.offset = 0;
        remora->cpg_right_pectrol.offset = 0;
        remora->cpg_left_pectrol.amplitude = 0;
        remora->cpg_right_pectrol.amplitude = 0;
        remora->cpg_left_pectrol.frequency = 0;
        remora->cpg_right_pectrol.frequency = 0;
    }

//    if (pec_front_mode == 0) {
//        if (pec_mode == 0) {
//            if (fabs(pec) > 5) {
//                if (pec > 0) {
//                    remora->cpg_left_pectrol.offset = -70;
//                    remora->cpg_right_pectrol.offset = -70;
//                } else {
//                    remora->cpg_left_pectrol.offset = 70;
//                    remora->cpg_right_pectrol.offset = 70;
//                }
//                remora->cpg_left_pectrol.amplitude = f_pec * pec_roll_amp * plus_count;
//                remora->cpg_right_pectrol.amplitude = f_pec * pec_roll_amp;
//                remora->cpg_left_pectrol.frequency = f_pec * pec_roll_freq;
//                remora->cpg_right_pectrol.frequency = f_pec * pec_roll_freq;
//            } else {
//                remora->cpg_left_pectrol.offset = 0;
//                remora->cpg_right_pectrol.offset = 0;
//                remora->cpg_left_pectrol.amplitude = 0;
//                remora->cpg_right_pectrol.amplitude = 0;
//                remora->cpg_left_pectrol.frequency = 0;
//                remora->cpg_right_pectrol.frequency = 0;
//            }
//        } else if (pec_mode == 1) {
//            if (fabs(pec) > 5) {
//                remora->cpg_left_pectrol.offset = 20;
//                remora->cpg_right_pectrol.offset = 20      ;
//                remora->cpg_left_pectrol.amplitude = f_pec * pec_roll_amp * plus_count;
//                remora->cpg_right_pectrol.amplitude = f_pec * pec_roll_amp;
//                remora->cpg_left_pectrol.frequency = f_pec * pec_roll_freq;
//                remora->cpg_right_pectrol.frequency = f_pec * pec_roll_freq;
//            } else {
//                remora->cpg_left_pectrol.offset = 0;
//                remora->cpg_right_pectrol.offset = 0;
//                remora->cpg_left_pectrol.amplitude = 0;
//                remora->cpg_right_pectrol.amplitude = 0;
//                remora->cpg_left_pectrol.frequency = 0;
//                remora->cpg_right_pectrol.frequency = 0;
//            }
//
//        } else if (pec_mode == -1) {
//            if (fabs(pec) > 5) {
//                remora->cpg_left_pectrol.offset = 60;
//                remora->cpg_right_pectrol.offset = -60;
//                remora->cpg_left_pectrol.amplitude = f_pec * pec_roll_amp * plus_count;
//                remora->cpg_right_pectrol.amplitude = f_pec * pec_roll_amp;
//                remora->cpg_left_pectrol.frequency = f_pec * pec_roll_freq;
//                remora->cpg_right_pectrol.frequency = f_pec * pec_roll_freq;
//            } else {
//                remora->cpg_left_pectrol.offset = 0;
//                remora->cpg_right_pectrol.offset = 0;
//                remora->cpg_left_pectrol.amplitude = 0;
//                remora->cpg_right_pectrol.amplitude = 0;
//                remora->cpg_left_pectrol.frequency = 0;
//                remora->cpg_right_pectrol.frequency = 0;
//            }
//        }
//    } else{
//        if (1) {
//            remora->cpg_left_pectrol.offset = 20;
//            remora->cpg_right_pectrol.offset = 20;
//            remora->cpg_left_pectrol.amplitude = 0;
//            remora->cpg_right_pectrol.amplitude = 0;
//            remora->cpg_left_pectrol.frequency = 0;
//            remora->cpg_right_pectrol.frequency = 0;
//        }
//    }



/*   if (suck_mode == 0){
        remora->bumper_disc.mode = BUMPER_MODE_POS;
        remora->bumper_disc.set = 0;
    } else if (suck_mode ==-1){
        remora->bumper_disc.mode = BUMPER_MODE_POS;
        remora->bumper_disc.set = -135;
    }
    else if (suck_mode==1){
        remora->bumper_disc.mode = BUMPER_MODE_POS;
        remora->bumper_disc.set = 135;
    }
*/

    if (neck_mode<=0){
//        remora->bumper_neck_pitch.dynamic=200;
//        remora->bumper_neck_pitch.mode = BUMPER_MODE_POS;
//        remora->bumper_neck_pitch.set = neck*3-20;
        Motor_setPositionDegree(&remora->motor_neck_pitch, neck*2-20);
        Motor_setPositionDegree(&remora->motor_neck_roll, disc_control*4-175);
    }
}

void Motion_Network1(fish_t *remora){
    static int32_t cnt=0;
    static int32_t stop_cnt = 0;
    static uint32_t stop_status = 0;

    float head_angle, body_angle;
    float target_control_angle;
    double pec = channel_value[1];
    double front = channel_value[2];
    double yaw = 0;
    double disc_control=channel_value[5];
    int neck_mode = -1;
    double pitch_ang = lround(channel_value[0]);
    int pec_mode = lround(channel_value[6]);
    //   int suck_mode = lround(channel_value[7]);

    double f_pec = fabs(pec);
    double pec_pitch_amp = 0.6;
    double pec_pitch_freq = 0.03;
    double pec_roll_amp = 0.6;
    double pec_roll_freq = 0.04;
    double tail_freq = 0.03;
    double tail_offset = 0.04;
    uint16_t dist_mm;
//    time_measure_record_start("motion_1");
//    time_measure_record_start("motion_get_angle");
    head_angle = IMU_GetPitch(&IMU_Head);
    body_angle = IMU_GetPitch(&IMU_Body);
//    time_measure_record_stop("motion_get_angle");
//
    time_measure_func("motion_get_dist", atk_ms53l1m_modbus_get_data(1, &dist_mm));
//
    time_measure_record_start("motion_get_net");
    Network_Send(fabs(head_angle/180.0*3.1415), (float)dist_mm*(float)cos(head_angle/180.0*3.1415)*0.65f/50.0f);
    Network_Read(&target_control_angle);
    time_measure_record_stop("motion_get_net");

//    time_measure_record_stop("motion_1");

//    Motor_setPositionDegree(&remora->motor_neck_pitch, pitch_ang * 1.35);
//    Motor_setPositionDegree(&remora->motor_disc, pitch_ang*1.35);

    if (dist_mm<100 && (fabs(head_angle)<30)){
        stop_cnt++;
        if (stop_cnt > 200){
            stop_status = 1;
            stop_cnt = 200;
        }

    } else{
        stop_cnt--;
        if (stop_cnt<1){
            stop_status = 0;
            stop_cnt = 1;
        }
    }
    if (stop_status){
        remora->bumper_neck_pitch.dynamic = 0.3;
        remora->bumper_neck_pitch.mode = BUMPER_MODE_POS;
        remora->bumper_neck_pitch.set = 0;
    }
    else{
        remora->bumper_neck_pitch.dynamic = 5;
        remora->bumper_neck_pitch.mode = BUMPER_MODE_POS;
        remora->bumper_neck_pitch.set = (int32_t)(target_control_angle*180.0/3.14159)*7-30;
    }


    if (fabs(front) < 5){
        remora->cpg_tail1.amplitude = 0;
        remora->cpg_tail2.amplitude = 0;
        remora->cpg_tail1.offset = 0;
        remora->cpg_tail2.offset = 0;

    } else{
        remora->cpg_tail1.amplitude = 18.0;
        remora->cpg_tail2.amplitude = 36.0;
        remora->cpg_tail1.offset = -yaw * tail_offset;
        remora->cpg_tail2.offset = -yaw * tail_offset*1.5;
//      remora->cpg_tail1.offset = yaw * 0;
//      remora->cpg_tail2.offset = yaw * 0;
    }
    remora->cpg_tail1.frequency = front*tail_freq;
    remora->cpg_tail2.frequency = front*tail_freq;

    if (pec_mode == 0){
        if (fabs(pec) > 5){
            if (pec>0){
                remora->cpg_left_pectrol.offset = -80;
                remora->cpg_right_pectrol.offset = -80;
            } else{
                remora->cpg_left_pectrol.offset = 80;
                remora->cpg_right_pectrol.offset = 80;
            }
            remora->cpg_left_pectrol.amplitude = f_pec*pec_pitch_amp;
            remora->cpg_right_pectrol.amplitude = f_pec*pec_pitch_amp;
            remora->cpg_left_pectrol.frequency = f_pec*pec_pitch_freq;
            remora->cpg_right_pectrol.frequency = f_pec*pec_pitch_freq;
        } else{
            remora->cpg_left_pectrol.offset = 0;
            remora->cpg_right_pectrol.offset = 0;
            remora->cpg_left_pectrol.amplitude = 0;
            remora->cpg_right_pectrol.amplitude = 0;
            remora->cpg_left_pectrol.frequency = 0;
            remora->cpg_right_pectrol.frequency = 0;
        }
    } else{
        if (fabs(pec) > 5){
            remora->cpg_left_pectrol.offset = 0;
            remora->cpg_right_pectrol.offset = 0;
            remora->cpg_left_pectrol.amplitude = f_pec*pec_roll_amp;
            remora->cpg_right_pectrol.amplitude = f_pec*pec_roll_amp;
            remora->cpg_left_pectrol.frequency = f_pec*pec_roll_freq;
            remora->cpg_right_pectrol.frequency = f_pec*pec_roll_freq;
        } else{
            remora->cpg_left_pectrol.offset = 0;
            remora->cpg_right_pectrol.offset = 0;
            remora->cpg_left_pectrol.amplitude = 0;
            remora->cpg_right_pectrol.amplitude = 0;
            remora->cpg_left_pectrol.frequency = 0;
            remora->cpg_right_pectrol.frequency = 0;
        }
    }


}

void Motion_SetModeSimple(int mode){
    motion_mode = mode;
}


/**
 * 负责运动状态的储存与管理
 */
void Motion_Update(fish_t *remora){

    if (motion_mode == MOTION_MODE_REMOTE)
        Motion_Remote_Control(remora);
    if  (motion_mode == MOTION_MODE_NETWORK1)
        Motion_Network1(remora);
}



