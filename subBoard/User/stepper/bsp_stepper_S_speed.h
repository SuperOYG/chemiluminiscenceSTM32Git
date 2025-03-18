#ifndef __BSP_STEPPER_S_SPEED_H
#define	__BSP_STEPPER_S_SPEED_H

#include "stm32f1xx_hal.h"
//#include "./usart/bsp_debug_usart.h"
//#include "bsp_stepper_init.h"
#include <stdbool.h>

//定时器实际时钟频率为：72MHz/(TIM_PRESCALER+1)
//其中 高级定时器的 频率为72MHz,其他定时器为36MHz
//72/(2)=36Mhz
//具体需要的频率可以自己计算
#define TIM_PRESCALER               (2 - 1)
// 定义定时器周期，输出比较模式周期设置为0xFFFF
#define TIM_PERIOD                   0xFFFF
/****************************************************************/
#define HIGH GPIO_PIN_SET       //高电平
#define LOW  GPIO_PIN_RESET     //低电平

#define ON  LOW                 //开
#define OFF HIGH                //关

#define CW  HIGH                //顺时针
#define CCW LOW                 //逆时针

//控制使能引脚
#define MOTOR_DIR_PIN GPIO_PIN_14
#define MOTOR_DIR_GPIO_PORT GPIOB
#define MOTOR_EN_PIN GPIO_PIN_15
#define MOTOR_EN_GPIO_PORT GPIOB
#define MOTOR_PUL_PIN GPIO_PIN_8
#define MOTOR_PUL_GPIO_PORT GPIOA

/* 带参宏，可以像内联函数一样使用 */
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
	__IO uint8_t  status;             /* 状态 */
	__IO uint8_t  dir;                /* 方向 */
	__IO uint32_t pos;                /* 位置 */
  __IO uint16_t pluse_time;         /* 脉冲时间 */
}Stepper_Typedef;

/* S加减速所用到的参数 */
typedef struct {
  __IO int32_t Vo;                  /* 初始速度 */
  __IO int32_t Vt;                  /* 末速度 */
  __IO int32_t AccelTotalStep;      /* 加速总步数 */
  __IO int32_t DecPoint;            /* 开始减速的步数 */
  __IO int32_t TotalStep;           /* 完整曲线总步数 */
  __IO int32_t INC_AccelTotalStep;  /* 加加速度步数 */
  __IO int32_t Dec_AccelTotalStep;  /* 减加速度步数 */
  __IO float   Form[FORM_LEN];       /* S加减速 速度表 */
}SpeedCalc_TypeDef;

/* 电机速度决策中的四个状态 */
typedef enum {
  STOP = 0U,                        /* 停止状态 */
  ACCEL,                            /* 加速状态 */
  UNIFORM,                          /* 匀速状态 */
  DECEL,                            /* 减速状态 */
}StateEnum_TypeDef;

/*频率相关参数*/
#define T1_FREQ_3               (SystemCoreClock / TIM_PRESCALER)//频率ft值
/*电机单圈参数*/
#define STEP_ANGLE						1.8f									           //步进电机的步距角 单位：度
#define FSPR_3              		(360.0f / 1.8f)                  //步进电机的一圈所需脉冲数
			
#define MICRO_STEP        		4          				             //细分器细分数   原来是32细分。
#define SPR_3               		(FSPR_3 * MICRO_STEP)              //细分后一圈所需脉冲数

#define CONVER(speed)         (float)(speed * SPR_3 / 60.0f)     //根据电机转速（r/min），计算电机步速（step/s）
  
#define MIN_SPEED             (float)(T1_FREQ_3 / 65535U)        //最低频率/速度

extern Stepper_Typedef Stepper;

bool Stepper_Move_S(int16_t start_speed, int16_t end_speed, float acc_time, int32_t step);

void Speed_Decision(void);

//#define TIM_TimeBaseStructure htim1;

#endif 
