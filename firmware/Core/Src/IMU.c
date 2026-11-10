
#include "IMU.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "string.h"
#include "stdio.h"
#include "pid.h"
#include "motor.h"

IMU_dev IMU_Head;
IMU_dev IMU_Body;
/**
 * 数据储存
 */


void Uart_JY901CallBack(IMU_dev *dev){

    if (dev->ucRxBuffer_jy901[0] != 0x55) {//数据头不对，则重新开始寻找0x55数据头
        dev->ucRxCnt_jy901=0;
    }
    else if(dev->ucRxCnt_jy901 >= 11)
    {
        switch(dev->ucRxBuffer_jy901[1])
        {
            case 0x50:	memcpy(&dev->stcTime, &dev->ucRxBuffer_jy901[2], 8);break;//memcpy为编译器自带的内存拷贝函数，需引用"string.h"，将接收缓冲区的字符拷贝到数据共同体里面，从而实现数据的解析。
            case 0x51:	memcpy(&dev->stcAcc, &dev->ucRxBuffer_jy901[2], 8);break;
            case 0x52:	memcpy(&dev->stcGyro, &dev->ucRxBuffer_jy901[2], 8);break;
            case 0x53:	memcpy(&dev->stcAngle, &dev->ucRxBuffer_jy901[2], 8);break;
            case 0x54:	memcpy(&dev->stcMag, &dev->ucRxBuffer_jy901[2], 8);break;
            case 0x55:	memcpy(&dev->stcDStatus, &dev->ucRxBuffer_jy901[2], 8);break;
            case 0x56:	memcpy(&dev->stcPress, &dev->ucRxBuffer_jy901[2], 8);break;
            case 0x57:	memcpy(&dev->stcLonLat, &dev->ucRxBuffer_jy901[2], 8);break;
            case 0x58:	memcpy(&dev->stcGPSV, &dev->ucRxBuffer_jy901[2], 8);break;
        }
        dev->ucRxCnt_jy901=0;
    }

    HAL_UART_Receive_IT(dev->uart, &dev->ucRxBuffer_jy901[dev->ucRxCnt_jy901++], 1);
}

float IMU_GetRoll(IMU_dev *dev){
    return (float)dev->stcAngle.Angle[0]/32768*180;
}

float IMU_GetPitch(IMU_dev *dev){
    return (float)dev->stcAngle.Angle[1]/32768*180;
}

float IMU_GetYaw(IMU_dev *dev){
    return (float)dev->stcAngle.Angle[2]/32768*180;
}

void IMU_reset_acc(IMU_dev *dev){
    uint8_t command[3] = {0xFF, 0xAA, 0x67};
    HAL_UART_Transmit(dev->uart, command, 3, 1000);
}

void IMU_Init(){
    IMU_Body.uart = &huart1;
    IMU_Head.uart = &huart2;
    IMU_reset_acc(&IMU_Head);
    HAL_UART_Receive_IT(IMU_Head.uart, &IMU_Head.ucRxBuffer_jy901[IMU_Head.ucRxCnt_jy901++], 1);
    HAL_UART_Receive_IT(IMU_Body.uart, &IMU_Body.ucRxBuffer_jy901[IMU_Body.ucRxCnt_jy901++], 1);
}