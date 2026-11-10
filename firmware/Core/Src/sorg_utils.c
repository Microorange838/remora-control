/*************************************************************
 * @file sorg_utils.c
 * @author MicroOrange
 * @brief 
 * @date 2022/12/27
 *************************************************************/
#include "usart.h"
#include "tim.h"
#include "stm32h7xx.h"

__IO uint64_t tick_us_4000000000;

void tick_us_init(){
    HAL_TIM_Base_Start_IT(&htim5);
    tick_us_4000000000 = 0;
}

void tick_us_callback(){
    tick_us_4000000000 += 1;
}

uint64_t tick_get_us(){
    return __HAL_TIM_GET_COUNTER(&htim5) + tick_us_4000000000 * 4000000000;
}