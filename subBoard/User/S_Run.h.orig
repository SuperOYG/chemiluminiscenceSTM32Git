#ifndef __RUN_H
#define	__RUN_H

#include "stm32f10x.h"

//电机正向运动
#define Dir_forward     0x01
//电机反向运动
#define Dir_reverse     0x02 

#define CP1_GPIO_PORT    	GPIOA			              
#define CP1_GPIO_CLK 	    RCC_APB2Periph_GPIOA		
#define CP1_GPIO_PIN		  GPIO_Pin_0	

#define DIR1_GPIO_PORT    	GPIOA			              
#define DIR1_GPIO_CLK 	    RCC_APB2Periph_GPIOA		
#define DIR1_GPIO_PIN		    GPIO_Pin_1	

#define EN1_GPIO_PORT    	GPIOB			              
#define EN1_GPIO_CLK 	    RCC_APB2Periph_GPIOB		
#define EN1_GPIO_PIN		  GPIO_Pin_9	

#define REF1_GPIO_PORT    	GPIOB			              
#define REF1_GPIO_CLK 	    RCC_APB2Periph_GPIOB		
#define REF1_GPIO_PIN		    GPIO_Pin_8	

#define CP2_GPIO_PORT    	GPIOB			              
#define CP2_GPIO_CLK 	    RCC_APB2Periph_GPIOB		
#define CP2_GPIO_PIN		  GPIO_Pin_6

#define DIR2_GPIO_PORT    	GPIOB			              
#define DIR2_GPIO_CLK 	    RCC_APB2Periph_GPIOB		
#define DIR2_GPIO_PIN		    GPIO_Pin_7	

#define EN2_GPIO_PORT    	GPIOB			              
#define EN2_GPIO_CLK 	    RCC_APB2Periph_GPIOB		
#define EN2_GPIO_PIN		  GPIO_Pin_5	

#define REF2_GPIO_PORT    	GPIOA			              
#define REF2_GPIO_CLK 	    RCC_APB2Periph_GPIOA		
#define REF2_GPIO_PIN		    GPIO_Pin_8	

#define CP3_GPIO_PORT    	GPIOA			              
#define CP3_GPIO_CLK 	    RCC_APB2Periph_GPIOA		
#define CP3_GPIO_PIN		  GPIO_Pin_7

#define DIR3_GPIO_PORT    	GPIOB			              
#define DIR3_GPIO_CLK 	    RCC_APB2Periph_GPIOB		
#define DIR3_GPIO_PIN		    GPIO_Pin_0	

#define EN3_GPIO_PORT    	GPIOA			              
#define EN3_GPIO_CLK 	    RCC_APB2Periph_GPIOA		
#define EN3_GPIO_PIN		  GPIO_Pin_6	

#define REF3_GPIO_PORT    	GPIOA			              
#define REF3_GPIO_CLK 	    RCC_APB2Periph_GPIOA		
#define REF3_GPIO_PIN		    GPIO_Pin_5	

/* 直接操作寄存器的方法控制IO */
#define	digitalHi(p,i)		 {p->BSRR=i;}	 //输出为高电平		
#define digitalLo(p,i)		 {p->BRR=i;}	 //输出低电平
#define digitalToggle(p,i) {p->ODR ^=i;} //输出反转状态

#define CP1_TOGGLE    digitalToggle(CP1_GPIO_PORT,CP1_GPIO_PIN)
#define CP1_ON		    digitalLo(CP1_GPIO_PORT,CP1_GPIO_PIN)
#define CP1_OFF		    digitalHi(CP1_GPIO_PORT,CP1_GPIO_PIN)

#define DIR1_OFF		  digitalLo(DIR1_GPIO_PORT,DIR1_GPIO_PIN)
#define DIR1_ON		    digitalHi(DIR1_GPIO_PORT,DIR1_GPIO_PIN)

#define EN1_ON		    digitalLo(EN1_GPIO_PORT,EN1_GPIO_PIN)
#define EN1_OFF		    digitalHi(EN1_GPIO_PORT,EN1_GPIO_PIN)

#define REF1_OFF		    digitalLo(REF1_GPIO_PORT,REF1_GPIO_PIN)
#define REF1_ON		  digitalHi(REF1_GPIO_PORT,REF1_GPIO_PIN)

#define CP2_TOGGLE    digitalToggle(CP2_GPIO_PORT,CP2_GPIO_PIN)
#define CP2_ON		    digitalLo(CP2_GPIO_PORT,CP2_GPIO_PIN)
#define CP2_OFF		    digitalHi(CP2_GPIO_PORT,CP2_GPIO_PIN)

#define DIR2_OFF		  digitalLo(DIR2_GPIO_PORT,DIR2_GPIO_PIN)
#define DIR2_ON		    digitalHi(DIR2_GPIO_PORT,DIR2_GPIO_PIN)

#define EN2_ON		    digitalLo(EN2_GPIO_PORT,EN2_GPIO_PIN)
#define EN2_OFF		    digitalHi(EN2_GPIO_PORT,EN2_GPIO_PIN)

#define REF2_OFF		    digitalLo(REF2_GPIO_PORT,REF2_GPIO_PIN)
#define REF2_ON		  digitalHi(REF2_GPIO_PORT,REF2_GPIO_PIN)

#define CP3_TOGGLE    digitalToggle(CP3_GPIO_PORT,CP3_GPIO_PIN)
#define CP3_ON		    digitalLo(CP3_GPIO_PORT,CP3_GPIO_PIN)
#define CP3_OFF		    digitalHi(CP3_GPIO_PORT,CP3_GPIO_PIN)

#define DIR3_OFF		  digitalLo(DIR3_GPIO_PORT,DIR3_GPIO_PIN)
#define DIR3_ON		    digitalHi(DIR3_GPIO_PORT,DIR3_GPIO_PIN)

#define EN3_ON		    digitalLo(EN3_GPIO_PORT,EN3_GPIO_PIN)
#define EN3_OFF		    digitalHi(EN3_GPIO_PORT,EN3_GPIO_PIN)

#define REF3_OFF		    digitalLo(REF3_GPIO_PORT,REF3_GPIO_PIN)
#define REF3_ON		  digitalHi(REF3_GPIO_PORT,REF3_GPIO_PIN)

#define ON  1
#define OFF 0

#define KEY_ON	1
#define KEY_OFF	0

//限位开关  P7
#define Zero7_GPIO_PORT   GPIOB	 
#define Zero7_GPIO_CLK    RCC_APB2Periph_GPIOB
#define Zero7_GPIO_PIN		GPIO_Pin_12

//限位开关  P13
#define Zero13_GPIO_PORT   GPIOB	 
#define Zero13_GPIO_CLK    RCC_APB2Periph_GPIOB
#define Zero13_GPIO_PIN		 GPIO_Pin_11

//限位开关  P8
#define Zero8_GPIO_PORT   GPIOB	 
#define Zero8_GPIO_CLK    RCC_APB2Periph_GPIOB
#define Zero8_GPIO_PIN		GPIO_Pin_10

//限位开关  P9
#define Zero9_GPIO_PORT   GPIOB	 
#define Zero9_GPIO_CLK    RCC_APB2Periph_GPIOB
#define Zero9_GPIO_PIN		GPIO_Pin_1

//限位开关  P11
#define Zero11_GPIO_PORT   GPIOB	 
#define Zero11_GPIO_CLK    RCC_APB2Periph_GPIOB
#define Zero11_GPIO_PIN		 GPIO_Pin_14

//限位开关  P12
#define Zero12_GPIO_PORT   GPIOB	 
#define Zero12_GPIO_CLK    RCC_APB2Periph_GPIOB
#define Zero12_GPIO_PIN		 GPIO_Pin_13

//限位开关  P20
#define Zero20_GPIO_PORT   GPIOB	 
#define Zero20_GPIO_CLK    RCC_APB2Periph_GPIOB
#define Zero20_GPIO_PIN		 GPIO_Pin_15

uint8_t Single_Scan(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin);

void GPIO_CP(void);
void GPIO_ConfigOut(void);
void GPIO_ConfigIn(void);

void ReadParameter(void);
void ReadSingleParameter(unsigned char R1,unsigned char R2,unsigned char R3,unsigned char R4,unsigned char R5,unsigned char R6,unsigned char R7);
void WriteSingleParameter(unsigned char R1,unsigned char R2,unsigned char R3,unsigned char R4,unsigned char R5,unsigned char R6,unsigned char R7);

#endif 
