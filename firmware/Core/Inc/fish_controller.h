/*************************************************************
 * @file fish.h
 * @author MicroOrange
 * @brief 鱼体部分控制
 * @date 2022.1.13
 *************************************************************/

#ifndef INC_113REBUILD_FISH_CONTROLLER_H
#define INC_113REBUILD_FISH_CONTROLLER_H

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

#define LIDAR_OFFSET 3.0f //mm
#define LIDAR_RELATE_OFFSET 85.0f //mm

typedef struct type_remora_fish{
    motor_type motor_right_pectrol;
    motor_type motor_left_pectrol;
    motor_type motor_tail1;
    motor_type motor_tail2;
    motor_type motor_neck_roll;
    motor_type motor_neck_yaw;
    motor_type motor_neck_pitch;
    motor_type motor_disc;
    pid_type pid_yaw;
    pid_type pid_roll_speed;
    pid_type pid_roll_mid;
    pid_type pid_pitch;
    cpg_node_t cpg_right_pectrol;
    cpg_node_t cpg_left_pectrol;
    cpg_node_t cpg_tail1;
    cpg_node_t cpg_tail2;
    bumper_isotropy_t bumper_neck_pitch;
    bumper_isotropy_t bumper_disc;
    bumper_anisotropy_t bumper_neck_roll;
} fish_t;


enum {
    TAIL_MODE_FRONT,
    TAIL_MODE_BACK,
    TAIL_MODE_STOP
};

enum {
    NECK_MODE_IMU_YAW,
    NECK_MODE_IMU_PITCH,
    NECK_MODE_ANGLE
};

enum {
    PEC_MODE_IMU_ROLL_SPEED,
    PEC_MODE_IMU_ROLL_MID,
    PEC_MODE_FRONT
};

enum {
    PEC_RUN_MODE_FORWARD,
    PEC_RUN_MODE_BACK
};

void Fish_Init();
void CPG_SetPara(int para_index, double value);
void Fish_CPGUpdate();
void Fish_CPGToMotor();

extern fish_t remora;

#endif //INC_113REBUILD_FISH_CONTROLLER_H
