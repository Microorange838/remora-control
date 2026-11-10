//
// Created by 86159 on 2023/10/30.
//

#ifndef REMORARL_LIDAR_H
#define REMORARL_LIDAR_H

/* Includes ------------------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32h7xx_hal.h"
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define DOUT_Pin GPIO_PIN_8
#define DOUT_GPIO_Port GPIOA

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */
/* USER CODE BEGIN Private defines */
#define USART_BUF_SIZE              64  /*USART_BUFFER_SIZE*/

void Lidar_RX_Proc(uint8_t *buf, uint32_t len);
/* USER CODE END Private defines */

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif
#endif //REMORARL_LIDAR_H
