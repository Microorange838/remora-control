#include "stdio.h"
#include "lidar.h"
#include "usart.h"

#define BW_TFLidar_DATA_Len             9
#define BW_TFLidar_DATA_HEAD            0x59

uint16_t lidar_dist;



void Lidar_RX_Proc(uint8_t *buf, uint32_t len)
{
    uint32_t i = 0;
    uint8_t chk_cal = 0;
    uint16_t dist = 0;

    if(BW_TFLidar_DATA_Len == len)
    {
        if((BW_TFLidar_DATA_HEAD == buf[0])&&(BW_TFLidar_DATA_HEAD == buf[1]))
        {
            for(i = 0; i < (BW_TFLidar_DATA_Len - 1); i++)
            {
                chk_cal += buf[i];
            }

            if(chk_cal == buf[BW_TFLidar_DATA_Len - 1])
            {
                dist 		 = buf[2] | (buf[3] << 8);

                lidar_dist = dist;

            }
        }
    }
}

uint16_t Lidar_Get_Dist(){
    return lidar_dist;
}

void Lidar_Init(){
    __HAL_UART_ENABLE_IT(&huart7, UART_IT_IDLE);
}
