//
// Created by Gmail on 2023/12/16.
//

#ifndef REMORARL_P8864_SENSOR_H
#define REMORARL_P8864_SENSOR_H
#include "stdint.h"
#include "usart.h"

typedef struct {
    uint8_t head;
    uint8_t response_type;
    uint16_t len;
    uint32_t distance[64];
    uint32_t amp[64];
    uint8_t crc[4];
} p8864_data_t;

extern UART_HandleTypeDef *p8864_uart_handle;
extern p8864_data_t pdata;


void p8864_init();
void p8864_read_data();
void p8864_dats_show(p8864_data_t *p8864_data);
#endif //REMORARL_P8864_SENSOR_H
