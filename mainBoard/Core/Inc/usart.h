/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "stdio.h"
/* USER CODE END Includes */

#include "string.h"
extern UART_HandleTypeDef huart1;

extern UART_HandleTypeDef huart2;

extern UART_HandleTypeDef huart3;

/* USER CODE BEGIN Private defines */
extern uint8_t  RecieveBuffer[1];//暂存接收到的字符
extern uint8_t  Rx_end;  //指令接收完成标志位
extern uint8_t  RxLine;  //rxbuf接收的数据长度为:RxLine+1;
extern uint8_t  rxbuf[50];//收到的数据存放处







#define QUEUE_SIZE         10      // 队列最多存储10条指令
#define MAX_CMD_LEN        50      // 单条指令最大长度

// 指令队列元素结构体
typedef struct {
    uint8_t data[MAX_CMD_LEN];     // 指令数据
    uint8_t len;                    // 指令长度
} QueueElement;

// 串口实例管理结构体
typedef struct {
    // 接收状态机
    enum { RX_IDLE, RX_STARTED } rxState;
    uint8_t rxBuffer[MAX_CMD_LEN];  // 接收缓冲区
    uint8_t rxIndex;                // 当前接收位置
    
    // 环形队列
    QueueElement cmdQueue[QUEUE_SIZE];
    uint8_t queueHead;
    uint8_t queueTail;
    uint8_t queueCount;
} UART_Instance;










/* USER CODE END Private defines */

void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);
void MX_USART3_UART_Init(void);

/* USER CODE BEGIN Prototypes */
int fputc(int ch, FILE *f);
int fgetc(FILE *f);
void Usart_SendByte(UART_HandleTypeDef *huart,uint8_t ch);

void Usart2_SendByte(uint8_t data);
void Usart3_SendByte(uint8_t data);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

