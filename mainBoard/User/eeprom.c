#include "eeprom.h"
#include "stdio.h"
#include "usart.h"
#include "can.h"
#include "work_interface.h"
#include "string.h"

#define ADDR_24LCxx_Write 0xA2
#define ADDR_24LCxx_Read 0xA3

#define BufferSize 256

uint8_t WriteBuffer[BufferSize] = {0};
uint8_t ReadBuffer[BufferSize] = {0};

extern I2C_HandleTypeDef hi2c1;
uint8_t I2c_Buf_Write[40];
uint8_t I2c_Buf_Read[40];

uint8_t TextBufferX[8] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x44, 0x55, 0x55};



//首页地址范围：0x00~0x1F  保存电机相关参数：   以机器左数第一组为第一组
//-----------------------------电机复位参数补偿----------------------------------
uint16_t  equipment_addres_1 = 0x0000;               // 第一组：参数地址
uint16_t  equipment_parament_1;                      //出仓复位补偿

uint16_t  equipment_addres_2 = 0x0002;               //参数地址
uint16_t  equipment_parament_2;                      //磁棒复位补偿

uint16_t  equipment_addres_3 = 0x0004;               //参数地址
uint16_t  equipment_parament_3;                      //震荡复位补偿

uint16_t  equipment_addres_4 = 0x0006;               //第二组：参数地址
uint16_t  equipment_parament_4;                      //出仓复位补偿

uint16_t  equipment_addres_5 = 0x0008;               //参数地址
uint16_t  equipment_parament_5;                      //磁棒复位补偿

uint16_t  equipment_addres_6 = 0x000A;               //参数地址：
uint16_t  equipment_parament_6;                      // 震荡复位补偿

uint16_t  equipment_addres_7 = 0x000C;               //第三组 参数地址：
uint16_t  equipment_parament_7;                      //出仓复位补偿

uint16_t  equipment_addres_8 = 0x000E;               //参数地址：
uint16_t  equipment_parament_8;                      //磁棒复位补偿

uint16_t  equipment_addres_9 = 0x0010;               //参数地址：
uint16_t  equipment_parament_9;    					 //震荡复位补偿
//-- 下面这个参数对应上述的参数正负号，标记一个出仓参数的正负号首地址，其余参数的地址对应加1 即可--
uint16_t  equipment_addres_1_1 = 0x0012;             //第一组：参数地址
uint16_t  equipment_parament_1_1;                      //出仓复位补偿






extern uint16_t  equipment_addres_1_1;             //第一组：参数地址
extern uint16_t  equipment_parament_1_1;                      //出仓复位补偿

extern uint16_t  equipment_addres_1_2;             //参数地址
extern uint16_t  equipment_parament_1_2;                      //磁棒复位补偿

extern uint16_t  equipment_addres_1_3;               //参数地址
extern uint16_t  equipment_parament_1_3;                      //震荡复位补偿

extern uint16_t  equipment_addres_1_4;               //第二组：参数地址
extern uint16_t  equipment_parament_1_4;                      //出仓复位补偿

extern uint16_t  equipment_addres_1_5;               //参数地址
extern uint16_t  equipment_parament_1_5;                      //磁棒复位补偿

extern uint16_t  equipment_addres_1_6;               //参数地址：
extern uint16_t  equipment_parament_1_6;                      // 震荡复位补偿

extern uint16_t  equipment_addres_1_7;               //第三组 参数地址：
extern uint16_t  equipment_parament_1_7;                      //出仓复位补偿

extern uint16_t  equipment_addres_1_8;               //参数地址：
extern uint16_t  equipment_parament_1_8;                      //磁棒复位补偿

extern uint16_t  equipment_addres_1_9;               //参数地址：
extern uint16_t  equipment_parament_1_9;    					 //震荡复位补偿



//首页地址范围：0x20~0x1F  保存单元组的状态：例如 ：是处于刺破、吸液和震荡状态。

uint16_t    Unit1_Status_addres = 0x0020; 

uint16_t  	Unit2_Status_addres = 0x0021;

uint16_t    Unit3_Status_addres = 0x0020;




void convertNumberToArray(uint32_t number, uint8_t* array);

//-----------------------------电机复位参数补偿----------------------------------


void I2CTest(void);

//从当前地址往后获取两个字节数据
uint16_t I2C_get2Byte(uint16_t equipment_addres)
{
    uint8_t readBuffer[2] = {0};
    uint16_t a;
    HAL_I2C_Mem_Read(&hi2c1, ADDR_24LCxx_Read, equipment_addres, I2C_MEMADD_SIZE_16BIT, readBuffer, 2, 1000);

    a = readBuffer[0] * 256 + readBuffer[1];
    printf("para1=0x%02X\r\n", readBuffer[0]);
    printf("para2=0x%02X\r\n", readBuffer[1]);
    return a;
}

//获取当前地址位的数据值
uint8_t I2C_getByte(uint16_t equipment_addres)
{
    uint8_t readBuffer[1] = {0};
    uint16_t a;
    HAL_I2C_Mem_Read(&hi2c1, ADDR_24LCxx_Read, equipment_addres, I2C_MEMADD_SIZE_16BIT, readBuffer, 1, 1000);
    a = readBuffer[0];
    printf("para1=0x%02X\r\n", readBuffer[0]);
    return a;
}

//往当前地址位写入一个字节数据
void I2C_writeByte(uint16_t equipment_addres, uint8_t num)
{
    uint8_t writerbuf[1] = {num};
    HAL_I2C_Mem_Write(&hi2c1, ADDR_24LCxx_Write, equipment_addres, I2C_MEMADD_SIZE_16BIT, writerbuf, 1, 100);
}

//往当前地址位开始写入2个字节数据
void I2C_write2Byte(uint16_t equipment_addres, uint16_t num)
{
//    int a = num;
    printf("%d", num);
    uint8_t writerbuf[2];
    if (num > 256)
    {
        writerbuf[0] = num / 256;
        writerbuf[1] = (uint8_t)num % 256;
    }
    else
    {
        writerbuf[0] = 0;
        writerbuf[1] = num;
    }
    HAL_I2C_Mem_Write(&hi2c1, ADDR_24LCxx_Write, equipment_addres, I2C_MEMADD_SIZE_16BIT, writerbuf, 2, 100);
    printf("Wpara1=0x%02X\r\n", writerbuf[0]);
    printf("Wpara2=0x%02X\r\n", writerbuf[1]);
}

//发送电机复位校准参数
void sendMotorParams(u16 address,u8 boardCode,u8 moterCode) {
    uint8_t TextBuffer[8];
    uint8_t hexArray[5];
    u8 symbolAddress=address/2+equipment_addres_1_1; //参数正负号地址位：
    TextBuffer[0]=0x04; //标志位
//    printf("当前获取参数正负号的地址位置%d\r\n",symbolAddress);
    TextBuffer[3]=I2C_getByte(symbolAddress);//正负号
    equipment_parament_1 = I2C_get2Byte(address);
//    printf("当前获取参数的地址位%d\r\n",address);
//    printf("第一组出仓电机补偿=%d\r\n",equipment_parament_1);
    convertNumberToArray(equipment_parament_1,hexArray);
    for (int i = 0; i < 4; i++)  //只取转换后数组的后四位。
    {
        TextBuffer[i+4] = hexArray[i+1];
    }
    TextBuffer[1]=boardCode;//板号：
    TextBuffer[2]=moterCode;//电机号:
    CAN_senddata(&hcan, TextBuffer, boardCode);//发送给1号子板
    HAL_Delay(100);//发送间隔时间：
	
    for(int i=0; i<8; i++) { 
        Usart2_SendByte(TextBuffer[i]);
    }
	//延时等待当前发送任务完成。
	
    if(boardCode==0x01) {
        while(HAL_GPIO_ReadPin(Can_Signal_1_GPIO_Port,Can_Signal_1_Pin)==1) {
            HAL_Delay(10);
        }
    }
    else if(boardCode==0x02) {
        while(HAL_GPIO_ReadPin(Can_Signal_2_GPIO_Port,Can_Signal_2_Pin)==1) {
            HAL_Delay(10);
        }	
    } else if(boardCode==0x03) {
        while(HAL_GPIO_ReadPin(Can_Signal_3_GPIO_Port,Can_Signal_3_Pin)==1) {
            HAL_Delay(10);
        }
    }
		HAL_Delay(100);
}

//获取保存在I2C内的系统参数。
void Read_equipment_parament()
{
	sendMotorParams(equipment_addres_1,0x01,0x01);
	sendMotorParams(equipment_addres_1+2,0x01,0x02);
	sendMotorParams(equipment_addres_1+4,0x01,0x03);
	sendMotorParams(equipment_addres_1+6,0x02,0x01);
	sendMotorParams(equipment_addres_1+8,0x02,0x02);
	sendMotorParams(equipment_addres_1+10,0x02,0x03);
	sendMotorParams(equipment_addres_1+12,0x03,0x01);
	sendMotorParams(equipment_addres_1+14,0x03,0x02);
	sendMotorParams(equipment_addres_1+16,0x03,0x03);
}
//I2C 测试---

void I2CTest() {
    printf("\r\n***************I2C Example*******************************\r\n");
    uint32_t i;
    uint8_t j;
    for(i = 0; i < 256; i++)
    {
        WriteBuffer[i] = i;    /* WriteBuffer init */
        printf("0x%02X ", WriteBuffer[i]);
        if(i % 16 == 15)
        {
            printf("\n\r");
        }
    }
    /* wrinte date to EEPROM */
    for (j = 0; j < 32; j++)
    {
        if(HAL_I2C_Mem_Write(&hi2c1, ADDR_24LCxx_Write, 8*j, I2C_MEMADD_SIZE_8BIT, WriteBuffer+8*j, 8, 100) == HAL_OK)
        {
            printf("\r\n EEPROM 24C02 Write Test OK \r\n");
        }
        else
        {
            printf("\r\n EEPROM 24C02 Write Test False \r\n");
        }
    }
    /* read date from EEPROM */
    HAL_I2C_Mem_Read(&hi2c1, ADDR_24LCxx_Read, 0, I2C_MEMADD_SIZE_8BIT, ReadBuffer, BufferSize, 1000);
    for(i = 0; i < 256; i++)
    {
        printf("0x%02X  ",ReadBuffer[i]);
        if(i%16 == 15)
        {
            printf("\n\r");
        }
    }

    if(memcmp(WriteBuffer,ReadBuffer,BufferSize) == 0 ) /* check date */
    {
        printf("\r\n EEPROM 24C02 Read Test OK\r\n");
    }
    else
    {
        printf("\r\n EEPROM 24C02 Read Test False\r\n");
    }
}




/**
*   @brief:  将0~99999 之间的任意数值转换成[3x,3x,3x,3x,3x]的形式。
*   @param:  void
*   @return: void
*   @author Hang
*   @date: 2023-05-28
*/

void convertNumberToArray(uint32_t number, uint8_t* array) {
    array[0] = (number / 10000) + 0x30;  // 万位
    array[1] = ((number / 1000) % 10) + 0x30;  // 千位
    array[2] = ((number / 100) % 10) + 0x30;  // 百位
    array[3] = ((number / 10) % 10) + 0x30;  // 十位
    array[4] = (number % 10) + 0x30;  // 个位
}












