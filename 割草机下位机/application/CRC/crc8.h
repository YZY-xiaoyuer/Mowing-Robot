/**
  ******************************************************************************
  * @file    crc8.h
  * @author  WenXianlei
  * @version V0.0.1
  * @date    2017/8/11
  * @brief   宝乐割草机上下位机通信CRC校验
  ******************************************************************************        
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CRC8_H
#define __CRC8_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "user_header.h"
     


/* USER CODE BEGIN Private defines */


/* USER CODE END Private defines */


/* USER CODE BEGIN Prototypes */

/* 逐次计算法 */
extern uint8_t crc8(uint8_t *data, uint32_t length);  
extern uint8_t crc8_itu(uint8_t *data, uint32_t length);  
extern uint8_t crc8_rohc(uint8_t *data, uint32_t length);  
extern uint8_t crc8_maxim(uint8_t *data, uint32_t length);

/* 查表法 */
extern uint8_t crc8_lut(uint8_t *data, uint32_t length);  
extern uint8_t crc8_itu_lut(uint8_t *data, uint32_t length);  
extern uint8_t crc8_rohc_lut(uint8_t *data, uint32_t length);  
extern uint8_t crc8_maxim_lut(uint8_t *data, uint32_t length);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /* __CRC8_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
