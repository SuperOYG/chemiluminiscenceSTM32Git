#ifndef __WORK_H
#define __WORK_H
#include "main.h"



#define u8 uint8_t
#define u16 uint16_t
#define u32 uint32_t

#define Frame_header 0x101

#define LED_H HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_SET)
#define LED_L HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_RESET)


void test(void);
void init(void);


void u16ToHexArray(u16 num, u8* hexArray);
void CAN_senddata(CAN_HandleTypeDef *hcan, uint8_t Data[8], u32 id);


#endif

