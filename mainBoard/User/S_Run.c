
#include "S_Run.h"
//#include "./i2c/bsp_i2c_ee.h"
//#include "bsp_usart.h"
//#include "./Adc/bsp_adc.h"
//#include "bsp_SysTick.h"
#include "stm32f10x.h"

/*******************************************************************************************************/
u8 I2c_Buf_Write[2];
u8 I2c_Buf_Read[2];
unsigned int Cresult[8]= {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
/*******************************************************************************************************/


/************************************************************************/
void EEPROM_write(unsigned char a1,unsigned char a2,unsigned char addr)
{
//	I2c_Buf_Write[0] = a1;
//	I2c_Buf_Write[1] = a2;
//	SysTick_Delay_Ms(10);
//	I2C_EE_BufferWrite(I2c_Buf_Write,addr,2);
//  SysTick_Delay_Ms(10);
}

void EEPROM_read(unsigned char addr)
{
//	SysTick_Delay_Ms(10);
//	I2C_EE_BufferRead(I2c_Buf_Read, addr, 2);
//	SysTick_Delay_Ms(10);
}

void ReadParameter()
{

}

/*******************************   写入参数     *******************************/
void WriteSingleParameter(unsigned char R1,unsigned char R2,unsigned char R3,unsigned char R4,unsigned char R5,unsigned char R6,unsigned char R7)
{

}
/*******************************  读出单个参数   *******************************/
void ReadSingleParameter(unsigned char R1,unsigned char R2,unsigned char R3,unsigned char R4,unsigned char R5,unsigned char R6,unsigned char R7)
{
//	I2C_EE_Init();			 // I2C 外设初(AT24C02)始化
//	SysTick_Delay_Ms(10);
//	I2c_Buf_Read[0]=I2c_Buf_Read[1]=0;
//	Canshu_H = Canshu_L = 0;
}

//脉冲1输出管脚设置
void GPIO_CP1(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitStructure.GPIO_Pin = CP1_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(CP1_GPIO_PORT, &GPIO_InitStructure);
}

//脉冲1输出管脚设置
void GPIO_CP2(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitStructure.GPIO_Pin = CP2_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(CP2_GPIO_PORT, &GPIO_InitStructure);

}

//脉冲1输出管脚设置
void GPIO_CP3(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitStructure.GPIO_Pin = CP3_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(CP3_GPIO_PORT, &GPIO_InitStructure);
}

//输出管脚设置
void GPIO_ConfigOut(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitStructure.GPIO_Pin = DIR1_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(DIR1_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = EN1_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(EN1_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = REF1_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(REF1_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = DIR2_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(DIR2_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = EN2_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(EN2_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = REF2_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(REF2_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = DIR3_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(DIR3_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = EN3_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(EN3_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = REF3_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(REF3_GPIO_PORT, &GPIO_InitStructure);
}

void GPIO_ConfigIn(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitStructure.GPIO_Pin = Zero7_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(Zero7_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = Zero8_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(Zero8_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = Zero9_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(Zero9_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = Zero11_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(Zero11_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = Zero12_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(Zero12_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = Zero13_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(Zero13_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = Zero20_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(Zero20_GPIO_PORT, &GPIO_InitStructure);

}

uint8_t Single_Scan(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin)
{
    if(GPIO_ReadInputDataBit(GPIOx,GPIO_Pin) == OFF )
    {
        return 	OFF;
    }
    else
        return ON;
}
