#ifndef __BSP_STEPPER_S_SPEED_H
#define	__BSP_STEPPER_S_SPEED_H

#include "stm32f1xx_hal.h"
//#include "./usart/bsp_debug_usart.h"
//#include "bsp_stepper_init.h"
#include <stdbool.h>

//��ʱ��ʵ��ʱ��Ƶ��Ϊ��72MHz/(TIM_PRESCALER+1)
//���� �߼���ʱ���� Ƶ��Ϊ72MHz,������ʱ��Ϊ36MHz
//72/(2)=36Mhz
//������Ҫ��Ƶ�ʿ����Լ�����
#define TIM_PRESCALER               (2 - 1)
// ���嶨ʱ�����ڣ�����Ƚ�ģʽ��������Ϊ0xFFFF
#define TIM_PERIOD                   0xFFFF
/****************************************************************/
#define HIGH GPIO_PIN_SET       //�ߵ�ƽ
#define LOW  GPIO_PIN_RESET     //�͵�ƽ

#define ON  LOW                 //��
#define OFF HIGH                //��

#define CW  HIGH                //˳ʱ��
#define CCW LOW                 //��ʱ��

//����ʹ������
#define MOTOR_DIR_PIN GPIO_PIN_14
#define MOTOR_DIR_GPIO_PORT GPIOB
#define MOTOR_EN_PIN GPIO_PIN_15
#define MOTOR_EN_GPIO_PORT GPIOB
#define MOTOR_PUL_PIN GPIO_PIN_8
#define MOTOR_PUL_GPIO_PORT GPIOA

/* ���κ꣬��������������һ��ʹ�� */
#define MOTOR_EN(x)					HAL_GPIO_WritePin(MOTOR_EN_GPIO_PORT,MOTOR_EN_PIN,x)
#define MOTOR_PUL(x)				HAL_GPIO_WritePin(MOTOR_PUL_GPIO_PORT,MOTOR_PUL_PIN,x)
#define MOTOR_DIR(x)				HAL_GPIO_WritePin(MOTOR_DIR_GPIO_PORT,MOTOR_DIR_PIN,x)

#define MOTOR_EN_TOGGLE     		HAL_GPIO_TogglePin(MOTOR_EN_GPIO_PORT,MOTOR_EN_PIN)
#define MOTOR_PUL_TOGGLE    		HAL_GPIO_TogglePin(MOTOR_PUL_GPIO_PORT,MOTOR_PUL_PIN)

#define MOTOR_PUL_IRQn                  TIM1_CC_IRQn
#define MOTOR_PUL_IRQHandler            TIM1_CC_IRQHandler


#define MOTOR_TIM_IT_CCx                TIM_IT_CC1
#define MOTOR_TIM_FLAG_CCx              TIM_FLAG_CC1



#define FORM_LEN 	   4000

typedef struct {
	__IO uint8_t  status;             /* ״̬ */
	__IO uint8_t  dir;                /* ���� */
	__IO uint32_t pos;                /* λ�� */
  __IO uint16_t pluse_time;         /* ����ʱ�� */
}Stepper_Typedef;

/* S�Ӽ������õ��Ĳ��� */
typedef struct {
  __IO int32_t Vo;                  /* ��ʼ�ٶ� */
  __IO int32_t Vt;                  /* ĩ�ٶ� */
  __IO int32_t AccelTotalStep;      /* �����ܲ��� */
  __IO int32_t DecPoint;            /* ��ʼ���ٵĲ��� */
  __IO int32_t TotalStep;           /* ���������ܲ��� */
  __IO int32_t INC_AccelTotalStep;  /* �Ӽ��ٶȲ��� */
  __IO int32_t Dec_AccelTotalStep;  /* �����ٶȲ��� */
  __IO float   Form[FORM_LEN];       /* S�Ӽ��� �ٶȱ� */
}SpeedCalc_TypeDef;

/* ����ٶȾ����е��ĸ�״̬ */
typedef enum {
  STOP = 0U,                        /* ֹͣ״̬ */
  ACCEL,                            /* ����״̬ */
  UNIFORM,                          /* ����״̬ */
  DECEL,                            /* ����״̬ */
}StateEnum_TypeDef;

/*Ƶ����ز���*/
#define T1_FREQ_3               (SystemCoreClock / TIM_PRESCALER)//Ƶ��ftֵ
/*�����Ȧ����*/
#define STEP_ANGLE						1.8f									           //��������Ĳ���� ��λ����
#define FSPR_3              		(360.0f / 1.8f)                  //���������һȦ����������
			
#define MICRO_STEP        		4          				             //ϸ����ϸ����   ԭ����32ϸ�֡�
#define SPR_3               		(FSPR_3 * MICRO_STEP)              //ϸ�ֺ�һȦ����������

#define CONVER(speed)         (float)(speed * SPR_3 / 60.0f)     //���ݵ��ת�٣�r/min�������������٣�step/s��
  
#define MIN_SPEED             (float)(T1_FREQ_3 / 65535U)        //���Ƶ��/�ٶ�

extern Stepper_Typedef Stepper;

bool Stepper_Move_S(int16_t start_speed, int16_t end_speed, float acc_time, int32_t step);

void Speed_Decision(void);

//#define TIM_TimeBaseStructure htim1;

#endif 
