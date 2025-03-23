#ifndef __WORK_H
#define __WORK_H
#include "main.h"

#define u8 uint8_t
#define u16 uint16_t
#define u32 uint32_t
//帧ID表示当前接收的CAN子板帧ID;也表示当前程序适用的单元电路  定义左侧为单元电路1，上方为单元电路2，右侧为单元电路3。
//#define Frame_header 0x01
#define Frame_header 0x02
//#define Frame_header 0x03

#define LED_H HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_SET)
#define LED_L HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_RESET)

void test(void);
void init(void);
void getTemp(void);

#endif

