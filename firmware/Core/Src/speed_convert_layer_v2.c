/*************************************************************
 * @file speed_convert_layer_v2.c
 * @author MicroOrange
 * @brief 速度转换层
 * @date 2022.1.31
 *************************************************************/

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
#include "math.h"
#include "para_service.h"
#include "speed_convert_layer_v2.h"

speed_convert_layer_v2_t convertLayer;

void _Convert_UpdateToController(fish_t *remora){
    double lpec_front,rpec_front,lpec_dive,rpec_dive;
    double tail_front,tail_yaw;

    lpec_front = convertLayer.Vf * convertLayer.kf;
    rpec_front = convertLayer.Vf * convertLayer.kf;

    lpec_dive = convertLayer.Vd * convertLayer.kd;
    rpec_dive = convertLayer.Vd * convertLayer.kd;

//    if (convertLayer.Vf >= 0){
//        if (convertLayer.Vy >= 0)
//            rpec_front += convertLayer.Vy * convertLayer.ky;
//        else
//            lpec_front += convertLayer.Vy * convertLayer.ky;
//    } else{
//        if (convertLayer.Vy <= 0)
//            rpec_front += convertLayer.Vy * convertLayer.ky;
//        else
//            lpec_front += convertLayer.Vy * convertLayer.ky;
//    }

    lpec_dive -= convertLayer.Vr * convertLayer.kr;
    rpec_dive += convertLayer.Vr * convertLayer.kr;


    double l_dirction,r_dirction;
    double l_f = sqrt(lpec_front*lpec_front + lpec_dive*lpec_dive);
    double r_f = sqrt(rpec_front*rpec_front + rpec_dive*rpec_dive);

    if (l_f != 0 || r_f!=0){
        l_dirction = atan2(lpec_dive, lpec_front);
        r_dirction = atan2(rpec_dive, rpec_front);
    }
    else{
        l_dirction = 0;
        r_dirction = 0;
    }

    // 此处direction 为rad
    if (convertLayer.Vf > convertLayer.Vf_threshold){
        tail_front = (convertLayer.Vf-convertLayer.Vf_threshold) * convertLayer.kf_tail;
    } else
        tail_front = 0;

    tail_yaw = -(convertLayer.Vy) * convertLayer.ky_tail_offset;
    tail_front += convertLayer.Vy * convertLayer.ky_tail_front;


    remora->cpg_left_pectrol.amplitude = l_f * convertLayer.k_pec_amp;
    remora->cpg_left_pectrol.frequency = 1 * convertLayer.k_pec_freq;
    remora->cpg_left_pectrol.offset = l_dirction/3.14*180;

    remora->cpg_right_pectrol.amplitude = r_f * convertLayer.k_pec_amp;
    remora->cpg_right_pectrol.frequency = 1 * convertLayer.k_pec_freq;
    remora->cpg_right_pectrol.offset = r_dirction/3.14*180;

    if (fabs(tail_front) <= 1e-3){
        remora->cpg_tail1.amplitude = 0;
        remora->cpg_tail2.amplitude = 0;
    } else{
        remora->cpg_tail1.amplitude = convertLayer.k_tail1_amp;
        remora->cpg_tail2.amplitude = convertLayer.k_tail2_amp;
    }
    remora->cpg_tail1.frequency = tail_front * convertLayer.k_tail1_freq;
    remora->cpg_tail2.frequency = tail_front * convertLayer.k_tail2_freq;
    remora->cpg_tail1.offset = tail_yaw;
    remora->cpg_tail2.offset = tail_yaw * convertLayer.k_tail1_2_offset;

    remora->bumper_neck_pitch.speed = convertLayer.Vp;

}

/**
 * 通过速度转换层为鱼体设置一个速度
 * @param remora 鱼体结构体指针
 * @param Vf
 * @param Vd
 * @param Vy
 * @param Vr
 * @param Vp
 */
void ConvertLayer_Update(fish_t *remora,double Vf,double Vd,double Vy,double Vr,double Vp){
    convertLayer.Vf = Vf;
    convertLayer.Vd = Vd;
    convertLayer.Vy = Vy;
    convertLayer.Vr = Vr;
    convertLayer.Vp = Vp;
    _Convert_UpdateToController(remora);
}

void ConvertLayer_Init(){
    convertLayer.kf = 1;
    convertLayer.kd = 1;
    convertLayer.ky = 1;
    convertLayer.kr = 1;
    convertLayer.kp = 1;
    convertLayer.Vf_threshold = 0.5;
    convertLayer.kf_tail = 1;
    convertLayer.Vy_threshold = 0;
    convertLayer.ky_tail_offset = 3;
    convertLayer.ky_tail_front = 0.3;
    convertLayer.k_pec_amp = 30;
    convertLayer.k_pec_freq = 1;
    convertLayer.k_tail1_amp = 15;
    convertLayer.k_tail2_amp = 25;
    convertLayer.k_tail1_freq = 1;
    convertLayer.k_tail2_freq = 1;
    convertLayer.k_tail1_2_offset = 0.8;

}

void ConvertLayer_Setpara(int para_index, double value){
    switch (para_index) {
        case PARA_CONVERT_kf:
            convertLayer.kf = value;
            break;
        case PARA_CONVERT_kd:
            convertLayer.kd = value;
            break;
        case PARA_CONVERT_ky:
            convertLayer.ky = value;
            break;
        case PARA_CONVERT_kr:
            convertLayer.kr = value;
            break;
        case PARA_CONVERT_kp:
            convertLayer.kp = value;
            break;
        case PARA_CONVERT_Vf_threshold:
            convertLayer.Vf_threshold = value;
            break;
        case PARA_CONVERT_kf_tail:
            convertLayer.kf_tail = value;
            break;
        case PARA_CONVERT_Vy_threshold:
            convertLayer.Vy_threshold = value;
            break;
        case PARA_CONVERT_ky_tail_offset:
            convertLayer.ky_tail_offset = value;
            break;
        case PARA_CONVERT_ky_tail_front:
            convertLayer.ky_tail_front = value;
            break;
        case PARA_CONVERT_k_pec_amp:
            convertLayer.k_pec_amp = value;
            break;
        case PARA_CONVERT_k_pec_freq:
            convertLayer.k_pec_freq = value;
            break;
        case PARA_CONVERT_k_tail1_amp:
            convertLayer.k_tail1_amp = value;
            break;
        case PARA_CONVERT_k_tail2_amp:
            convertLayer.k_tail2_amp = value;
            break;
        case PARA_CONVERT_k_tail1_freq:
            convertLayer.k_tail1_freq = value;
            break;
        case PARA_CONVERT_k_tail2_freq:
            convertLayer.k_tail2_freq = value;
            break;
        case PARA_CONVERT_k_tail1_2_offset:
            convertLayer.k_tail1_2_offset = value;
            break;
        default:
            break;
    }
}


