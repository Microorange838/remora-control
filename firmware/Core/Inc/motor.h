/*************************************************************
 * @file motor.h
 * @author MicroOrange
 * @brief 舵机控制抽象
 * @date 2022.1.13
 *************************************************************/

#ifndef MOTOR_H
#define MOTOR_H

#include "tim.h"
#include "usart.h"
#include "gpio.h"

#include "string.h"
#include "stdio.h"
#include "pid.h"

/**
 * 舵机对象
 * degree 角度（°） 不包含镜像效果
 *
 * timer 定时器
 * channel pwm通道
 * compare、gap 500+2000 pwm脉宽对应的CNT比较范围
 *
 * max degree 限幅角度
 *
 * mirrored 角度空间镜像标识
 * mirror center 角度空间镜像中点
 */
typedef struct {
	double pre_degree;
	double now_degree;

    TIM_HandleTypeDef timer;
    uint16_t channel;
    int pwm_compare_min;
    int pwm_gap;

    double max_degree;
    double degree_offset;

    int mirrored;
    double mirror_center;

}motor_type;


motor_type Motor_gen(TIM_HandleTypeDef timer, uint16_t channel, double max_degree, int compare_min, int compare_gap, int init_deg);
void Motor_setPositionPWM(motor_type *motor, int compare);
void Motor_setPositionDegree(motor_type *motor, double degree);



#endif