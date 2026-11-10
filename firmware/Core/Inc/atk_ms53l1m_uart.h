/**
 ****************************************************************************************************
 * @file        atk_ms53l1m_uart.h
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2022-06-21
 * @brief       ATK-MS53L1M模块UART接口驱动代码
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子 阿波罗 H743开发板
 * 在线视频:www.yuanzige.com
 * 技术论坛:www.openedv.com
 * 公司网址:www.alientek.com
 * 购买地址:openedv.taobao.com
 *
 ****************************************************************************************************
 */

#ifndef __ATK_MS53L1M_UART_H
#define __ATK_MS53L1M_UART_H

#include "sorg_utils.h"

/* 引脚定义 */
#define ATK_MS53L1M_UART_TX_GPIO_PORT           GPIOB
#define ATK_MS53L1M_UART_TX_GPIO_PIN            GPIO_PIN_10
#define ATK_MS53L1M_UART_TX_GPIO_AF             GPIO_AF7_USART3
#define ATK_MS53L1M_UART_TX_GPIO_CLK_ENABLE()   do{ __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)     /* PA口时钟使能 */

#define ATK_MS53L1M_UART_RX_GPIO_PORT           GPIOB
#define ATK_MS53L1M_UART_RX_GPIO_PIN            GPIO_PIN_11
#define ATK_MS53L1M_UART_RX_GPIO_AF             GPIO_AF7_USART3
#define ATK_MS53L1M_UART_RX_GPIO_CLK_ENABLE()   do{ __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)     /* PA口时钟使能 */

#define ATK_MS53L1M_UART_CLK_ENABLE()           do{ __HAL_RCC_USART3_CLK_ENABLE(); }while(0)    /* USART3 时钟使能 */

/* UART收发缓冲大小 */
#define ATK_MS53L1M_UART_RX_BUF_SIZE            128

/* 操作函数 */
void atk_ms53l1m_uart_send(uint8_t *dat, uint8_t len);  /* ATK-MS53L1M UART发送数据 */
void atk_ms53l1m_uart_rx_restart(void);                 /* ATK-MS53L1M UART重新开始接收数据 */
uint8_t *atk_ms53l1m_uart_rx_get_frame(void);           /* 获取ATK-MS53L1M UART接收到的一帧数据 */
uint16_t atk_ms53l1m_uart_rx_get_frame_len(void);       /* 获取ATK-MS53L0 UART接收到的一帧数据的长度 */
void atk_ms53l1m_uart_init();          /* ATK-MS53L1M UART初始化 */

void ATK_MS53L1M_UART_IRQHandler(void);

#endif
