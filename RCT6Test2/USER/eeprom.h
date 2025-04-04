#ifndef __EEPROM_H
#define	__EEPROM_H
#include "main.h"

extern uint8_t year;
extern uint8_t month;
extern uint8_t date;

extern uint8_t Time_h;
extern uint8_t Time_m;
extern uint8_t Time_s;

extern uint16_t  equipment_addres_1;                //参数地址
extern uint16_t  equipment_parament_1;			    //X轴方向：每个卡位间距

extern uint16_t  equipment_addres_2;             //参数地址
extern uint16_t  equipment_parament_2;		      //X轴初始偏移距离：也就是从原点到第一个测试点的距离

extern uint16_t  equipment_addres_3;            //参数地址
extern uint16_t  equipment_parament_3;		      //X轴退卡距离：卸卡钩与检测单元的距离

extern uint16_t  equipment_addres_4;            //参数地址
extern uint16_t  equipment_parament_4;		      //Y轴方向：卸卡距离

extern uint16_t  equipment_addres_5;            //参数地址
extern uint16_t  equipment_parament_5;		       //Y轴方向：扫描条形码距离

extern uint16_t  equipment_addres_6;          //参数地址
extern uint16_t  equipment_parament_6;		     // Y轴方向：测试点扫描点距离

extern uint16_t  equipment_addres_7;
extern uint16_t  equipment_parament_7;




extern uint16_t  Code_flag_address;            //条码开关
extern uint16_t  Code_flag;

extern uint16_t  equipment_addres_21 ;
extern uint32_t  equipment_parament_21;					//T1/C  范围

extern uint16_t Code_address[6];                      //条码地址
extern uint8_t Code_Parameter[6];                     //条码参数


void Read_equipment_parament(void);

void I2C_write2Byte(uint16_t equipment_addres, uint16_t num);
uint16_t I2C_get2Byte(uint16_t equipment_addres);

void I2C_writeByte(uint16_t equipment_addres, uint8_t num);
uint8_t I2C_getByte(uint16_t equipment_addres);

/************************************************************************/
#endif


