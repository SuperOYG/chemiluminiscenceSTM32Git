#include "eeprom.h"
#include "stdio.h"
#include "usart.h"
#include "can.h"
#include "work_interface.h"
#include "string.h"

//#define ADDR_24LCxx_Write 0xA0
//#define ADDR_24LCxx_Read 0xA1
#define ADDR_24LCxx_Write 0xA2
#define ADDR_24LCxx_Read 0xA3
#define BufferSize 256

//extern uint8_t WriteBuffer[BufferSize] ;
//extern uint8_t ReadBuffer[BufferSize] ;


uint8_t WriteBuffer[BufferSize] = {0};
uint8_t ReadBuffer[BufferSize] = {0};

extern I2C_HandleTypeDef hi2c1;
uint8_t I2c_Buf_Write[40];
uint8_t I2c_Buf_Read[40];

uint8_t TextBufferX[8] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x44, 0x55, 0x55};
extern uint16_t getNumb;


//3维 电机定位 : X轴-转盘电机，Y轴-测试方向,Z轴—卸卡电机。
uint16_t  equipment_addres_1 = 0x0000;               //参数地址
uint16_t  equipment_parament_1;                     //X轴方向：每个卡位间距

uint16_t  equipment_addres_2 = 0x0002;               //参数地址
uint16_t  equipment_parament_2;                     //X轴初始偏移距离：也就是将第一个测试卡槽移动到测试点的距离。

uint16_t  equipment_addres_3 = 0x0004;               //参数地址
uint16_t  equipment_parament_3;                     //X轴退卡距离：卸卡钩与检测单元的距离

uint16_t  equipment_addres_4 = 0x0006;              //参数地址
uint16_t  equipment_parament_4;                     //Y轴方向：测试点扫描点距离 

uint16_t  equipment_addres_5 = 0x0008;              //参数地址
uint16_t  equipment_parament_5;                     //Y轴方向：扫描条形码距离

uint16_t  equipment_addres_6 = 0x000A;               //参数地址：
uint16_t  equipment_parament_6;                     // Z轴方向：卸卡距离

uint16_t  equipment_addres_7 = 0x000C;               //参数地址：
uint16_t  equipment_parament_7;                     //Z轴卸卡电机的实时距离：

//uint16_t  equipment_addres_8 = 0x000F;               //参数地址：
//uint16_t  equipment_parament_8;                     //Z轴卸卡电机的实时距离：








uint16_t  CurvePeak_StartAdress1 = 0x0060;          //第一个曲线峰值起点地址
uint16_t  CurvePeak_Start1;                           //第一个曲线峰值起点值

uint16_t  CurvePeak_EndAdress1 = 0x0062;          //第一个曲线峰值终点地址
uint16_t  CurvePeak_End1;                           //第一个曲线峰值终点值

uint16_t  CurvePeak_StartAdress2 = 0x0064;          //第二个曲线峰值起点地址
uint16_t  CurvePeak_Start2;                           //第一个曲线峰值起点值

uint16_t  CurvePeak_EndAdress2 = 0x0066;          //第二个曲线峰值终点地址
uint16_t  CurvePeak_End2;                           //第一个曲线峰值终点值

uint16_t  CurvePeak_StartAdress3 = 0x0068;          //第三个曲线峰值起点地址
uint16_t  CurvePeak_Start3;                           //第一个曲线峰值起点值

uint16_t  CurvePeak_EndAdress3 = 0x006A;          //第三个曲线峰值终点地址
uint16_t  CurvePeak_End3;                           //第一个曲线峰值终点值

uint16_t  CurvePeak_StartAdress4 = 0x006C;          //第四个曲线峰值起点地址
uint16_t  CurvePeak_Start4;                           //第一个曲线峰值起点值

uint16_t  CurvePeak_EndAdress4 = 0x006E;          //第四个曲线峰值终点地址
uint16_t  CurvePeak_End4;                           //第一个曲线峰值终点值

uint16_t  CurvePeak_StartAdress5 = 0x0070;         //第五个个曲线峰值起点地址
uint16_t  CurvePeak_Start5;                           //第一个曲线峰值起点值

uint16_t  CurvePeak_EndAdress5 = 0x0072;          //第五个曲线峰值终点地址
uint16_t  CurvePeak_End5;                           //第一个曲线峰值终点值

uint16_t  CurvePeak_StartAdress6 = 0x0074;          //第六个曲线峰值起点地址
uint16_t  CurvePeak_Start6;                           //第一个曲线峰值起点值

uint16_t  CurvePeak_EndAdress6 = 0x0076;          //第六个曲线峰值终点地址
uint16_t  CurvePeak_End6;                           //第留个个曲线峰值终点值


uint16_t  referenceValueAdress = 0x0078;          //参考范围值地址
uint16_t  referenceValue;                           //参考范围值



uint16_t  IncubationTime_address = 0x00778;          //样本孵育时间地址
uint16_t  IncubationTime;                             //孵育时间参数



uint16_t  Code_flag_address = 0x0052;          //样本孵育时间地址
uint16_t  Code_flag;

u16  equipment_addres_21 = 0x0028;
u32  equipment_parament_21;                 //T1/C  范围

u16 Code_address[6] = {0x00A0,0x00A1,0x00A2,0x00A3,0x00A4,0x00A5};//条码地址
u8 Code_Parameter[6];                                       //条码参数

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
//        printf(">256");
        writerbuf[0] = num / 256;
        writerbuf[1] = (uint8_t)num % 256;
    }
    else
    {
//        printf("<256");
        writerbuf[0] = 0;
        writerbuf[1] = num;
    }
    HAL_I2C_Mem_Write(&hi2c1, ADDR_24LCxx_Write, equipment_addres, I2C_MEMADD_SIZE_16BIT, writerbuf, 2, 100);
    printf("Wpara1=0x%02X\r\n", writerbuf[0]);
    printf("Wpara2=0x%02X\r\n", writerbuf[1]);
}


//获取保存在I2C内的系统参数。
void Read_equipment_parament()
{
	I2C_write2Byte(equipment_addres_1, 480);
    equipment_parament_1 = I2C_get2Byte(equipment_addres_1);
    printf("横移间距=%d\r\n",equipment_parament_1);
	
	I2C_write2Byte(equipment_addres_2, 4800);
    equipment_parament_2 = I2C_get2Byte(equipment_addres_2);
	printf("初始距离=%d\r\n",equipment_parament_2);
	
	
	I2C_write2Byte(equipment_addres_4, 6000);
    equipment_parament_4 = I2C_get2Byte(equipment_addres_4);
	printf("测试距离=%d\r\n",equipment_parament_4);
}





//I2C 测试---

void I2CTest(){
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
















