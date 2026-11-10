     /*************************************************************
 * @file wireless_model.c
 * @author MicroOrange
 * @brief 无线模块通信部分
 * @date 2022.2.22
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
#include "speed_convert_layer_v2.h"
#include "math.h"
#include "motion_planner.h"
#include "para_service.h"
#include "aid_func.h"
#include "wireless_model.h"

/**
 * 串口接受缓存区组
 */
static unsigned char ucRxBuffer_wirelessmdl[256];
unsigned char dataBuffer[256];
float keyBuff[64],modeBuff[64],transitionBuff[64],cpgBuff[64],remoteBuff[64];

static uint8_t ucRxCnt=0;
static uint16_t len=0;
static uint8_t cmd=0;
static uint16_t crc16_result=0;
static uint8_t flag=0;
union res{
    float a[64];
    unsigned char tmpBuff[256];
}s;
union short_char{
    uint16_t b;
    unsigned char c[2];
}sc;


static unsigned short const wCRC16Table[256] = {
        0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
        0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
        0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
        0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
        0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
        0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
        0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
        0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
        0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
        0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
        0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
        0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
        0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
        0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
        0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
        0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
        0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
        0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
        0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
        0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
        0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
        0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
        0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
        0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
        0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
        0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
        0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
        0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
        0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
        0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
        0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
        0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040};


void CRC16(const unsigned char* pDataIn, int iLenIn, unsigned short* pCRCOut)
{
    unsigned short wResult = 0;
    unsigned short wTableNo = 0;
    int i = 0;
    for( i = 0; i < iLenIn; i++)
    {
        wTableNo = ((wResult & 0xff) ^ (pDataIn[i] & 0xff));
        wResult = ((wResult >> 8) & 0xff) ^ wCRC16Table[wTableNo];
    }

    *pCRCOut = wResult;
}

void Uart_WarelessModelCallBack(){
    if(ucRxBuffer_wirelessmdl[0] != 0xAA){
        ucRxCnt=0;
        len=0;
    }
    if(ucRxCnt==2 && ucRxBuffer_wirelessmdl[1] != 0x55){
        ucRxCnt=0;
        len=0;
    }
    if(ucRxCnt==3){
        cmd=ucRxBuffer_wirelessmdl[2];
    }
    if(ucRxCnt==5){
        sc.c[0]=ucRxBuffer_wirelessmdl[4];
        sc.c[1]=ucRxBuffer_wirelessmdl[3];
        len=sc.b;
        len=len+9;
    }
    if(ucRxCnt==len && len!=0){

        if(ucRxBuffer_wirelessmdl[len - 1] == 0x74 && ucRxBuffer_wirelessmdl[len - 2] == 0x03){
            memcpy(s.tmpBuff, &ucRxBuffer_wirelessmdl[5], (len - 9));
            CRC16(s.tmpBuff,(len-9),&crc16_result);
            sc.c[0]=ucRxBuffer_wirelessmdl[len - 3];
            sc.c[1]=ucRxBuffer_wirelessmdl[len - 4];
            if(crc16_result==sc.b){
                flag=1;
                len=len-9;
            }else{

                ucRxCnt=0;
                len=0;
            }
        }else{
            ucRxCnt=0;
            len=0;
        }
    }
//    printf("flag:%d", flag);
    if(flag){
        switch(cmd){
            case COMMAND_PARADATA: memcpy(modeBuff,s.a,len);break;
            case COMMAND_NONE: memcpy(transitionBuff,s.a,len);break;
            case COMMAND_NONE2: memcpy(cpgBuff,s.a,len);break;
            case COMMAND_KEYBOARD: memcpy(keyBuff,s.a,len);break;
            case COMMAND_REMOTE: memcpy(remoteBuff,s.a,len);break;
        }
        memset(ucRxBuffer_wirelessmdl,0,256);
        if (cmd == 0x01){
            for (int i = 0; i < PARA_MOTION_MODE_TEST2_Vy+1; ++i) {
                Para_SetPara(i, modeBuff[i]);
            }
        } else if (cmd == COMMAND_KEYBOARD){
            char a = keyBuff[0];
            if (a == 's')
                Para_SetPara(PARA_MOTION_MODE, 3);
            if (a == 'w'){
                Para_SetPara(PARA_MOTION_MODE, MOTION_MODE_FRONT);
                Para_SetPara(PARA_MOTION_MODE_TEST2_Vf, 10);
                Para_SetPara(PARA_MOTION_MODE_TEST2_Vd, 2);
            }
            if (a == 'a') {
                Para_SetPara(PARA_MOTION_MODE_TEST2_Vf, 40);
                Para_SetPara(PARA_MOTION_MODE_TEST2_Vd, 75);
                Para_SetPara(PARA_MOTION_MODE_TEST2_Vr, 40);
                Para_SetPara(PARA_MOTION_MODE_TEST2_Vp, -75);
                Para_SetPara(PARA_MOTION_MODE_TEST2_Vy, 2);
                Para_SetPara(PARA_MOTION_MODE, 4);
            }
            if (a == 'd') {
                Para_SetPara(PARA_MOTION_MODE_TEST2_Vf, 40);
                Para_SetPara(PARA_MOTION_MODE_TEST2_Vd, -75);
                Para_SetPara(PARA_MOTION_MODE_TEST2_Vr, 40);
                Para_SetPara(PARA_MOTION_MODE_TEST2_Vp, 75);
                Para_SetPara(PARA_MOTION_MODE_TEST2_Vy, 2);
                Para_SetPara(PARA_MOTION_MODE, 4);
            }
            if (a == 'x') {
                Para_SetPara(PARA_MOTION_MODE_TEST2_Vf, 40);
                Para_SetPara(PARA_MOTION_MODE_TEST2_Vd, -75);
                Para_SetPara(PARA_MOTION_MODE_TEST2_Vr, 40);
                Para_SetPara(PARA_MOTION_MODE_TEST2_Vp, -75);
                Para_SetPara(PARA_MOTION_MODE_TEST2_Vy, 3);
                Para_SetPara(PARA_MOTION_MODE, 4);
            }
            if (a == 'z'){
                Para_SetPara(PARA_MOTION_MODE, MOTION_MODE_DETACH_S1);
//                Para_SetPara(PARA_MOTION_MODE, MOTION_MODE_DETACH_S2);
//                delay_ms(2000);
//                Para_SetPara(PARA_MOTION_MODE_TEST2_Vf, 20);
//                Para_SetPara(PARA_MOTION_MODE_TEST2_Vd, 90);
//                Para_SetPara(PARA_MOTION_MODE_TEST2_Vr, 20);
//                Para_SetPara(PARA_MOTION_MODE_TEST2_Vp, 90);
//                Para_SetPara(PARA_MOTION_MODE_TEST2_Vy, 1.5);
//                Para_SetPara(PARA_MOTION_MODE, 4);
            }
            if (a == 'c'){
                Para_SetPara(PARA_MOTION_MODE, MOTION_MODE_DETACH_S2);
            }
            if (a == 'v'){
//                Para_SetPara(PARA_MOTION_MODE_TEST2_Vf, 40);
//                Para_SetPara(PARA_MOTION_MODE_TEST2_Vd, 80);
//                Para_SetPara(PARA_MOTION_MODE_TEST2_Vr, 40);
//                Para_SetPara(PARA_MOTION_MODE_TEST2_Vp, 80);
//                Para_SetPara(PARA_MOTION_MODE_TEST2_Vy, 2);
//                Para_SetPara(PARA_MOTION_MODE, 4);
                Para_SetPara(PARA_MOTION_MODE, MOTION_MODE_REMOTE);
            } else if (a>130){
                Para_SetPara(PARA_MOTION_MODE_TEST2_Vf, (a-130.0)/50.0);

            }
        } else if (cmd == COMMAND_REMOTE){
            for (int i = 0; i < 8; ++i) {
                Para_SetPara(PARA_REMOTE_CHANNEL_1+i, remoteBuff[i]);
            }

        }


        ucRxCnt=0;
        len=0;

    }
    flag = 0;
    memset(s.tmpBuff,0,256);
    HAL_UART_Receive_IT(&huart3, &ucRxBuffer_wirelessmdl[ucRxCnt++], 1);
}

void WirelessModel_Init(){
    HAL_UART_Receive_IT(&huart3, &ucRxBuffer_wirelessmdl[ucRxCnt++], 1);
}