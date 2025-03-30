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
extern uint8_t  RecieveBuffer[1];//�ݴ���յ����ַ�
extern uint8_t  Rx_end;  //ָ�������ɱ�־λ
extern uint8_t  RxLine;  //rxbuf���յ����ݳ���Ϊ:RxLine+1;
extern uint8_t  rxbuf[50];//�յ������ݴ�Ŵ�







#define QUEUE_SIZE         10      // �������洢10��ָ��
#define MAX_CMD_LEN        50      // ����ָ����󳤶�

// ָ�����Ԫ�ؽṹ��
typedef struct {
    uint8_t data[MAX_CMD_LEN];     // ָ������
    uint8_t len;                    // ָ���
} QueueElement;

// ����ʵ������ṹ��
typedef struct {
    // ����״̬��
    enum { RX_IDLE, RX_STARTED } rxState;
    uint8_t rxBuffer[MAX_CMD_LEN];  // ���ջ�����
    uint8_t rxIndex;                // ��ǰ����λ��
    
    // ���ζ���
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

