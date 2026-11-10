//
// Created by Gmail on 2023/12/16.
//

#include "usart.h"
#include "stdint.h"
#include "p8864_sensor.h"
#include "string.h"
#include "stdio.h"
#include "log.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"

UART_HandleTypeDef *p8864_uart_handle;
uint8_t p8864_print[1000];
p8864_data_t pdata;

uint8_t p8864_cmd_power_on[] = {0xF5, 0x40, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x9C, 0xD7, 0xD6, 0x91};
uint8_t p8864_cmd_get_data[] = {0xF5, 0x29, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xCB, 0xFA, 0xAB, 0xE7};

void p8864_power_on(){
    uint8_t buf[8];
    memset(buf, 0, 8);
    HAL_UART_Transmit(p8864_uart_handle, p8864_cmd_power_on, 14, 1000);
    HAL_UART_Receive(p8864_uart_handle, (uint8_t *)buf, 8, 1000);
    if ((buf[0] != 0xFA) || (buf[1] !=0x00)){
        Error_Handler();
    }
    LOGLN_INFO("tof", "p8864_ok");
}

void p8864_init(){
    p8864_uart_handle = &huart7;
    p8864_power_on();
}

void p8864_read_data(){
    taskENTER_CRITICAL();
    HAL_UART_Transmit(p8864_uart_handle, p8864_cmd_get_data, 14,500);
    taskEXIT_CRITICAL();
    HAL_UART_Receive_DMA(p8864_uart_handle, (uint8_t *)&pdata, sizeof(p8864_data_t));
}

void p8864_dats_show(p8864_data_t *p8864_data){
    memset(p8864_print, 0, 1000);

    for (int i = 0; i < 8; ++i) {
        for (int j = 0; j < 8; ++j) {
            sprintf(&p8864_print[strlen(p8864_print)], "%8lu", p8864_data->distance[i*8+j]);
        }

        sprintf(&p8864_print[strlen(p8864_print)], "\r\n");
    }
    for (int i = 0; i < 5; ++i) {
        sprintf(&p8864_print[strlen(p8864_print)], "\r\n");
    }
    HAL_UART_Transmit(&huart5, p8864_print, strlen(p8864_print), 1000);
}


