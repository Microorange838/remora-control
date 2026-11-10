/*************************************************************
 * @file wireless_model.h
 * @author MicroOrange
 * @brief 无线模块通信部分
 * @date 2022.2.22
 *************************************************************/

#ifndef INC_113REBUILD_WIRELESS_MODEL_H
#define INC_113REBUILD_WIRELESS_MODEL_H

enum {
    COMMAND_PARADATA=1,
    COMMAND_NONE,
    COMMAND_NONE2,
    COMMAND_KEYBOARD,
    COMMAND_REMOTE
};

void Uart_WarelessModelCallBack();
void Uart_WarelessModelCallBackTest();
void WirelessModel_Init();
#endif //INC_113REBUILD_WIRELESS_MODEL_H
