#ifndef __WORK_H
#define __WORK_H
#include "main.h"

#define u8 uint8_t
#define u16 uint16_t
#define u32 uint32_t
//֡ID��ʾ��ǰ���յ�CAN�Ӱ�֡ID;Ҳ��ʾ��ǰ�������õĵ�Ԫ��·  �������Ϊ��Ԫ��·1���Ϸ�Ϊ��Ԫ��·2���Ҳ�Ϊ��Ԫ��·3��
//#define Frame_header 0x01
#define Frame_header 0x02
//#define Frame_header 0x03

#define LED_H HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_SET)
#define LED_L HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_RESET)

void test(void);
void init(void);
void getTemp(void);

#endif

