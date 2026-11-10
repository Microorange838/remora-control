//
// Created by Gmail on 2022/1/14.
//

#include "stdio.h"
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

void delay_ms(int32_t nms)

{

    int32_t temp;

    SysTick->LOAD = 8000*nms;

    SysTick->VAL=0X00;//清空计数器

    SysTick->CTRL=0X01;//使能，减到零是无动做，采用外部时钟源

    do

    {

        temp=SysTick->CTRL;//读取当前倒计数值

    }

    while((temp&0x01)&&(!(temp&(1<<16))));//等待时间到达



    SysTick->CTRL=0x00; //关闭计数器

    SysTick->VAL =0X00; //清空计数器

}