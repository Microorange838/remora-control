/*************************************************************
 * @file motor.c
 * @author MicroOrange
 * @brief 舵机控制抽象
 * @date 2022.1.13
 *************************************************************/

#include "tim.h"
#include "usart.h"
#include "gpio.h"

#include "string.h"
#include "stdio.h"
#include "pid.h"
#include "motor.h"

motor_type Motor_gen(TIM_HandleTypeDef timer, uint16_t channel, double max_degree, int compare_min, int compare_gap, int init_deg){
    motor_type my_motor;
    my_motor.timer = timer;
    my_motor.channel = channel;
    my_motor.max_degree = max_degree;
    my_motor.pwm_gap = compare_gap;
    my_motor.pwm_compare_min = compare_min;

    my_motor.pre_degree = 0;
    my_motor.now_degree = 0;
    my_motor.mirrored = 0;

    my_motor.degree_offset = max_degree/2;
    Motor_setPositionDegree(&my_motor, init_deg);
    HAL_TIM_PWM_Start(&timer, channel);
    Motor_setPositionDegree(&my_motor, init_deg);
    return my_motor;
}

void Motor_setPositionPWM(motor_type *motor, int pos){
    if(pos > motor->pwm_compare_min+motor->pwm_gap)
        pos = motor->pwm_compare_min + motor->pwm_gap;
    if(pos < motor->pwm_compare_min)
        pos = motor->pwm_compare_min;
	__HAL_TIM_SET_COMPARE(&(motor->timer),motor->channel,pos);
}

void Motor_setPositionDegree(motor_type *motor, double degree){
    degree += motor->degree_offset;
    if (motor->mirrored){
        double real_degree = 2*motor->mirror_center - degree;
        if (real_degree>motor->max_degree)
            real_degree = motor->max_degree;
        if (real_degree<0)
            real_degree = 0;
        motor->pre_degree = motor->now_degree;
        motor->now_degree = degree;
        //mirror是一种对电机的封装 角度是用户操作的量
        Motor_setPositionPWM(motor, (int)((double)real_degree*(double)motor->pwm_gap/motor->max_degree+motor->pwm_compare_min));
    } else{
        if (degree>motor->max_degree)
            degree = motor->max_degree;
        if (degree<0)
            degree = 0;
        motor->pre_degree = motor->now_degree;
        motor->now_degree = degree;
        Motor_setPositionPWM(motor, (int)((double)degree*(double)motor->pwm_gap/motor->max_degree+motor->pwm_compare_min));

    }
}




