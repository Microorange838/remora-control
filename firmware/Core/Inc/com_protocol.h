/*************************************************************
 * @file com_protocol.h
 * @author MicroOrange
 * @brief 六耳猕猴通讯协议数据层
 * @date 2022.1.10
 *************************************************************/

#ifndef FISHCONTROLLERTEST_COM_PROTOCOL_H
#define FISHCONTROLLERTEST_COM_PROTOCOL_H

/**
 * 通用常量
 */
typedef unsigned int u32;
typedef unsigned char u8;

/**
 * 通讯相关常量
 */
#define COM_DEVICE_HEADER_1 0xA5
#define COM_DEVICE_HEADER_2 0x5A
#define COM_DEVICE_TAIL 0xA2

/**
 * 数据帧结构体
 */
typedef struct {
    u8 address[2];
    u8 command[2];
    u8 len;
    u8 data[256];
    u8 crc16[2];
}__attribute__((packed)) t_package;

/**
 * 通信设备对象
 *
 * 必须实现以下虚函数
 * processCallback 处理回调函数
 * sendByte 单字节发送函数 0表示成功
 *
 * 建议实现以下虚函数
 * timeoutStart 启动并清空计时器
 *
 * 方法
 * comReceiveByte 接受一个字节数据
 * comTimeout 超时终止一个包的接收
 */
typedef struct type_com_device{
    void (*processCallback)(t_package package);
    int (*sendByte)(u8 data);

    void (*timeoutStart)();
    void (*timeoutStop)();

    u8 status;
    int now_data_len;
    t_package package_buffer;
    int use_timeout;
} t_com_device;


/**
 * 设备接受状态常量
 */
#define COM_DEVICE_STATUS_IDLE 0x01
#define COM_DEVICE_STATUS_READY 0x00
#define COM_DEVICE_STATUS_HEADER_1 0x02
#define COM_DEVICE_STATUS_ADDRESS_1 0x04
#define COM_DEVICE_STATUS_ADDRESS_2 0x05
#define COM_DEVICE_STATUS_COMMAND_1 0x06
#define COM_DEVICE_STATUS_COMMAND_2 0x07
#define COM_DEVICE_STATUS_LEN 0x08
#define COM_DEVICE_STATUS_DATA 0x09
#define COM_DEVICE_STATUS_CRC_1 0x10
#define COM_DEVICE_STATUS_CRC_2 0x11
#define COM_DEVICE_STATUS_TAIL 0x12


void comReceiveByte(t_com_device *device, u8 data);
int comSendPackage(t_com_device *device, t_package *package);
t_com_device ComDevice();
void comTimeout(t_com_device *device);


#endif //FISHCONTROLLERTEST_COM_PROTOCOL_H
