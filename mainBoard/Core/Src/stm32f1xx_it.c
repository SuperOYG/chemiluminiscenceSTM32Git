/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usart.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern CAN_HandleTypeDef hcan;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line4 interrupt.
  */
void EXTI4_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_IRQn 0 */

  /* USER CODE END EXTI4_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(Can_Signal_1_Pin);
  /* USER CODE BEGIN EXTI4_IRQn 1 */

  /* USER CODE END EXTI4_IRQn 1 */
}

/**
  * @brief This function handles USB low priority or CAN RX0 interrupts.
  */
void USB_LP_CAN1_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN USB_LP_CAN1_RX0_IRQn 0 */

  /* USER CODE END USB_LP_CAN1_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan);
  /* USER CODE BEGIN USB_LP_CAN1_RX0_IRQn 1 */

  /* USER CODE END USB_LP_CAN1_RX0_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[9:5] interrupts.
  */
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */

  /* USER CODE END EXTI9_5_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(Can_Signal_2_Pin);
  HAL_GPIO_EXTI_IRQHandler(Can_Signal_3_Pin);
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */

  /* USER CODE END EXTI9_5_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */

  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */

  /* USER CODE END TIM4_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */

  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}

/* USER CODE BEGIN 1 */
uint8_t CurUartNum=2;

uint8_t  RecieveBuffer[1]={0};//�ݴ���յ����ַ�
uint8_t  RxLine=0; //��¼�������ݳ���
uint8_t  RxStart=0;   //��¼���ݿ�ʼ&������־λ
uint8_t  Rx_end=0;  //ָ�������ɱ�־λ
uint8_t  errorBuffer[]="\r\nerror\r\n"; //������ʾ
uint8_t  rxbuf[50];//�յ������ݴ�Ŵ�
uint8_t  UART_Code[7]; //����ֵ

extern uint8_t testStatus;
/**
 * @brief ���������ݽ���
 * @param  ���ں�
 * @retval void
 * @author smart_mode
 * @Time 2021��11��21��
*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	UNUSED(huart);
	if(huart==&huart2){  //����2 ����
		if (RecieveBuffer[0]==0xEE) // �ж�֡ͷ�Ƿ���ȷ
	{
		CurUartNum=2;
		RxStart=1;   //���տ�ʼ
		RxLine=0;    //���ճ�ʼ��ֵ��ַλΪ0;
	}
	if(RxStart==1){
	 if(RecieveBuffer[0]!=0xFF){
			rxbuf[RxLine++]=RecieveBuffer[0];
	 }else{
			rxbuf[RxLine]=RecieveBuffer[0];
			if(rxbuf[1]==0xB2&&rxbuf[2]==0x01){
				testStatus=1;
			}if(rxbuf[1]==0xB2&&rxbuf[2]==0x02){
				testStatus=0;
			}
			RxStart=0;  //���ս���
			Rx_end=1;    //�������
	 }
	}
	RecieveBuffer[0]=0;
	HAL_UART_Receive_IT(&huart2, (uint8_t *)RecieveBuffer, 1); //�ٴδ��ж�
	}
	else if(huart==&huart3){  //����3 ����
		if (RecieveBuffer[0]==0xEE) // �ж�֡ͷ�Ƿ���ȷ
	{
		CurUartNum=3;
		RxStart=1;   //���տ�ʼ
		RxLine=0;    //���ճ�ʼ��ֵ��ַλΪ0;
	}
	if(RxStart==1){
	 if(RecieveBuffer[0]!=0xFF){
			rxbuf[RxLine++]=RecieveBuffer[0];
	 }else{
			rxbuf[RxLine]=RecieveBuffer[0];
			RxStart=0;  //���ս���
			Rx_end=1;    //�������
	 }
	}
	RecieveBuffer[0]=0;
	HAL_UART_Receive_IT(&huart3, (uint8_t *)RecieveBuffer, 1); //�ٴδ��ж�
	}
//	else if(huart==&huart2){
//		uint8_t data;
//		data=RecieveBuffer[0];
//		if(data==0x2a){
//			RxStart=1;
//			RxLine=1;
//		}if(RxStart==1)
//		{
//			if(data !=0x0D){
//			UART_Code[RxLine++]=data;  //����ֵ��ֵ
//			}else {
//			RxStart=0;  //���ս���
//			Rx_end=0;    //�������
//			}
//		}
//		HAL_UART_Receive_IT(&huart2, (uint8_t *)RecieveBuffer, 1);//�ٴδ��ж�
//	}
}

/**
*@brief  CANͨѶ���жϻص�������
*/
CAN_TxHeaderTypeDef TXHeader;
CAN_RxHeaderTypeDef RXHeader;
uint8_t CanTXmessage[8];
uint8_t CanRXmessage[8];
uint32_t pTxMailbox = 0;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)//��������0�����жϻص�����
{
	if(hcan->Instance==CAN1)
	{
		HAL_CAN_GetRxMessage(hcan,CAN_FILTER_FIFO0,&RXHeader,CanRXmessage);//��ȡ����
		Rx_end=2;
	}
}


extern uint8_t CmdFlagBuf[12];
/**
*@brief  CANͨѶ IO�жϺ�����
*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	// �ر�ȫ���ж�
	__disable_irq();
	if(GPIO_Pin==Can_Signal_1_Pin)
	{
		CmdFlagBuf[4]=0x02;
	}else if(GPIO_Pin==Can_Signal_2_Pin){
		CmdFlagBuf[6]=0x02;
	}else if(GPIO_Pin==Can_Signal_3_Pin){
		CmdFlagBuf[8]=0x02;
	}
	HAL_UART_Transmit(&huart2,CmdFlagBuf,12,1000);
	// ��ȫ���ж�
	__enable_irq();
	
//	Usart2_SendByte(0xEE);
//	Usart2_SendByte(0xCC);
//	if(GPIO_Pin==Can_Signal_1_Pin)
//	{
//		Usart2_SendByte(0x01);	
//	}else if(GPIO_Pin==Can_Signal_2_Pin){
//		Usart2_SendByte(0x02);
//	}else if(GPIO_Pin==Can_Signal_3_Pin){
//		Usart2_SendByte(0x03);
//	}
//	Usart2_SendByte(0xDD);
//	Usart2_SendByte(0x0D);
//	Usart2_SendByte(0x0A);
	
}
/* USER CODE END 1 */
