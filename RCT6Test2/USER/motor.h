#ifndef __MOTOR_H
#define __MOTOR_H
#include "main.h"


#define KEY_ON    1
#define KEY_OFF   0

#define u8 uint8_t
#define u16 uint16_t
#define u32 uint32_t

//���ϸ�������޿���·��ĵ��ϸ����32���ҵĵ��1,2��4ϸ�֣� �𶯵����ϸ����8ϸ�֣�
//���ԣ���Ҫ����ԭ�г���Ļ����ϣ���һ���˶�������ϵ�������1,2 ��8 �����3��4ϸ�֣�
#define  Motor1_2Factor  8
#define  Motor3Factor  4

#define REF1_H		HAL_GPIO_WritePin(RBF1_GPIO_Port,RBF1_Pin,GPIO_PIN_SET)

#define EN1_H		HAL_GPIO_WritePin(EN4_GPIO_Port,EN4_Pin,GPIO_PIN_SET)
#define EN1_L		HAL_GPIO_WritePin(EN4_GPIO_Port,EN4_Pin,GPIO_PIN_RESET)
//DIR1_H ����  DIR_L ����
#define DIR1_L		HAL_GPIO_WritePin(DIR4_GPIO_Port,DIR4_Pin,GPIO_PIN_RESET)
#define DIR1_H		HAL_GPIO_WritePin(DIR4_GPIO_Port,DIR4_Pin,GPIO_PIN_SET)

#define CP1_TOGGLE	HAL_GPIO_TogglePin(CP4_GPIO_Port,CP4_Pin)
#define CP1_H		HAL_GPIO_WritePin(CP4_GPIO_Port,CP4_Pin,GPIO_PIN_SET)
#define CP1_L		HAL_GPIO_WritePin(CP4_GPIO_Port,CP4_Pin,GPIO_PIN_RESET)



#define REF2_H		HAL_GPIO_WritePin(RBF2_GPIO_Port,RBF2_Pin,GPIO_PIN_SET)

#define EN2_H		HAL_GPIO_WritePin(EN2_GPIO_Port,EN2_Pin,GPIO_PIN_SET)
#define EN2_L		HAL_GPIO_WritePin(EN2_GPIO_Port,EN2_Pin,GPIO_PIN_RESET)

#define DIR2_L		HAL_GPIO_WritePin(DIR2_GPIO_Port,DIR2_Pin,GPIO_PIN_RESET) 
#define DIR2_H		HAL_GPIO_WritePin(DIR2_GPIO_Port,DIR2_Pin,GPIO_PIN_SET)

#define CP2_TOGGLE	HAL_GPIO_TogglePin(CP2_GPIO_Port,CP2_Pin)
#define CP2_H		HAL_GPIO_WritePin(CP2_GPIO_Port,CP2_Pin,GPIO_PIN_SET)
#define CP2_L		HAL_GPIO_WritePin(CP2_GPIO_Port,CP2_Pin,GPIO_PIN_RESET)



#define REF3_H		HAL_GPIO_WritePin(RBF3_GPIO_Port,RBF3_Pin,GPIO_PIN_SET)

#define EN3_H		HAL_GPIO_WritePin(EN3_GPIO_Port,EN3_Pin,GPIO_PIN_SET)
#define EN3_L		HAL_GPIO_WritePin(EN3_GPIO_Port,EN3_Pin,GPIO_PIN_RESET)

#define DIR3_L		HAL_GPIO_WritePin(DIR3_GPIO_Port,DIR3_Pin,GPIO_PIN_RESET)
#define DIR3_H		HAL_GPIO_WritePin(DIR3_GPIO_Port,DIR3_Pin,GPIO_PIN_SET)

#define CP3_TOGGLE	HAL_GPIO_TogglePin(CP3_GPIO_Port,CP3_Pin)
#define CP3_H		HAL_GPIO_WritePin(CP3_GPIO_Port,CP3_Pin,GPIO_PIN_SET)
#define CP3_L		HAL_GPIO_WritePin(CP3_GPIO_Port,CP3_Pin,GPIO_PIN_RESET)



//#define REF4_H		HAL_GPIO_WritePin(RBF4_GPIO_Port,RBF4_Pin,GPIO_PIN_SET)

//#define EN4_H		HAL_GPIO_WritePin(EN4_GPIO_Port,EN4_Pin,GPIO_PIN_SET)
//#define EN4_L		HAL_GPIO_WritePin(EN4_GPIO_Port,EN4_Pin,GPIO_PIN_RESET)

//#define DIR4_H		HAL_GPIO_WritePin(DIR4_GPIO_Port,DIR4_Pin,GPIO_PIN_RESET)
//#define DIR4_L		HAL_GPIO_WritePin(DIR4_GPIO_Port,DIR4_Pin,GPIO_PIN_SET)

//#define CP4_TOGGLE	HAL_GPIO_TogglePin(CP4_GPIO_Port,CP4_Pin)
//#define CP4_H		HAL_GPIO_WritePin(CP4_GPIO_Port,CP4_Pin,GPIO_PIN_SET)
//#define CP4_L		HAL_GPIO_WritePin(CP4_GPIO_Port,CP4_Pin,GPIO_PIN_RESET)




void Motor_init(void);

void moter1_step(u32 distance,u8 flag);
void moter1_int(u16 step);

void moter2_step(u32 distance,u8 flag);
void moter2_step2(u32 distance,u8 flag);//distance = ����;

void moter2_int(u16 step);
void moter2_int2(u16 step);

void moter3_1step(u8 speed);
void Delay_moter3(u16 n);
void moter3_step(u32 distance,u8 flag);//distance = ����;
void moter3_int(u16 step);
/************************************************************************/
#endif
