#ifndef __MOTOR_H
#define __MOTOR_H
#include "main.h"


#define KEY_ON    1    
#define KEY_OFF   0

#define u8 uint8_t
#define u16 uint16_t
#define u32 uint32_t

//电机1-GPIO设置    （P10 J1和J4）  测试电机

#define REF1_H		HAL_GPIO_WritePin(RBF1_GPIO_Port,RBF1_Pin,GPIO_PIN_SET) 
 
#define EN1_H		HAL_GPIO_WritePin(EN1_GPIO_Port,EN1_Pin,GPIO_PIN_SET)
#define EN1_L		HAL_GPIO_WritePin(EN1_GPIO_Port,EN1_Pin,GPIO_PIN_RESET)
//DIR1_H 朝内  DIR_L 朝外
#define DIR1_H		HAL_GPIO_WritePin(DIR1_GPIO_Port,DIR1_Pin,GPIO_PIN_RESET)
#define DIR1_L		HAL_GPIO_WritePin(DIR1_GPIO_Port,DIR1_Pin,GPIO_PIN_SET)

#define CP1_TOGGLE	HAL_GPIO_TogglePin(CP1_GPIO_Port,CP1_Pin)
#define CP1_H		HAL_GPIO_WritePin(CP1_GPIO_Port,CP1_Pin,GPIO_PIN_SET)
#define CP1_L		HAL_GPIO_WritePin(CP1_GPIO_Port,CP1_Pin,GPIO_PIN_RESET)

//电机2-引脚设置     （P16 J2和J5）  转盘电机                          
#define REF2_H		HAL_GPIO_WritePin(RBF2_GPIO_Port,RBF2_Pin,GPIO_PIN_SET) 

#define EN2_H		HAL_GPIO_WritePin(EN2_GPIO_Port,EN2_Pin,GPIO_PIN_SET)
#define EN2_L		HAL_GPIO_WritePin(EN2_GPIO_Port,EN2_Pin,GPIO_PIN_RESET)

#define DIR2_H		HAL_GPIO_WritePin(DIR2_GPIO_Port,DIR2_Pin,GPIO_PIN_RESET)
#define DIR2_L		HAL_GPIO_WritePin(DIR2_GPIO_Port,DIR2_Pin,GPIO_PIN_SET)

#define CP2_TOGGLE	HAL_GPIO_TogglePin(CP2_GPIO_Port,CP2_Pin)
#define CP2_H		HAL_GPIO_WritePin(CP2_GPIO_Port,CP2_Pin,GPIO_PIN_SET)
#define CP2_L		HAL_GPIO_WritePin(CP2_GPIO_Port,CP2_Pin,GPIO_PIN_RESET)

//电机3-引脚设置     （P15 J3和J6）  卸卡电机
#define REF3_H		HAL_GPIO_WritePin(RBF3_GPIO_Port,RBF3_Pin,GPIO_PIN_SET) 

#define EN3_H		HAL_GPIO_WritePin(EN3_GPIO_Port,EN3_Pin,GPIO_PIN_SET)
#define EN3_L		HAL_GPIO_WritePin(EN3_GPIO_Port,EN3_Pin,GPIO_PIN_RESET)

#define DIR3_H		HAL_GPIO_WritePin(DIR3_GPIO_Port,DIR3_Pin,GPIO_PIN_RESET)
#define DIR3_L		HAL_GPIO_WritePin(DIR3_GPIO_Port,DIR3_Pin,GPIO_PIN_SET)

#define CP3_TOGGLE	HAL_GPIO_TogglePin(CP3_GPIO_Port,CP3_Pin)
#define CP3_H		HAL_GPIO_WritePin(CP3_GPIO_Port,CP3_Pin,GPIO_PIN_SET)
#define CP3_L		HAL_GPIO_WritePin(CP3_GPIO_Port,CP3_Pin,GPIO_PIN_RESET)

//电机4-引脚设置  暂时没用上
#define REF4_H		HAL_GPIO_WritePin(RBF4_GPIO_Port,RBF4_Pin,GPIO_PIN_SET) 

#define EN4_H		HAL_GPIO_WritePin(EN4_GPIO_Port,EN4_Pin,GPIO_PIN_SET)
#define EN4_L		HAL_GPIO_WritePin(EN4_GPIO_Port,EN4_Pin,GPIO_PIN_RESET)

#define DIR4_H		HAL_GPIO_WritePin(DIR4_GPIO_Port,DIR4_Pin,GPIO_PIN_RESET)
#define DIR4_L		HAL_GPIO_WritePin(DIR4_GPIO_Port,DIR4_Pin,GPIO_PIN_SET)

#define CP4_TOGGLE	HAL_GPIO_TogglePin(CP4_GPIO_Port,CP4_Pin)
#define CP4_H		HAL_GPIO_WritePin(CP4_GPIO_Port,CP4_Pin,GPIO_PIN_SET)
#define CP4_L		HAL_GPIO_WritePin(CP4_GPIO_Port,CP4_Pin,GPIO_PIN_RESET)


//限位开关





//试剂卡槽有、无卡状态检测
#define ON  0     
#define OFF 1

extern uint8_t Moter_speed;
 //电机接近开关信号：1：触发，0：未触发
extern u8 MotorX_speed ;  //横移方向：Y轴电机速度
extern u8 MotorY_speed ;  //测试方向：X轴方向滑块速度



void Ka1_InitBack(u8 delay,u8 direction); //测试电机 复位
void Ka2_InitBack(u8 delay);   //转盘电机  复位
void Ka3_InitBack(u8 delay);   //退卡电机  复位

void Ka2_InitBack2(u8 delay,u16 distance); //转盘电机 复位移动

void Ka1Moter_Step(u8 delay,u32 distance);
void Ka2Moter_Step(u8 delay,u32 distance);
void Ka3Moter_Step(u8 delay,u32 distance);

extern uint32_t x_now; //当前转盘位置
extern uint32_t y_now; //当前检测位置
extern uint32_t z_now; //卸卡电机位置

void Motor_init(void);
void addCard(void);
void dropCard(void);
void testCard(u8 testId);


void moter3_1step(u8 speed); 
void moter3_step(u32 distance);//distance = 坐标;
void Delay_moter3(u16 n);

void moter3_int(void);


/************************************************************************/
#endif 
