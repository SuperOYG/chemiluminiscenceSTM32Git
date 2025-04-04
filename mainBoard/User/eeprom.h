#ifndef __EEPROM_H
#define	__EEPROM_H
#include "main.h"

extern uint16_t  equipment_addres_1;                //参数地址
extern uint16_t  equipment_parament_1;

extern uint16_t  equipment_addres_2;             //参数地址
extern uint16_t  equipment_parament_2;

extern uint16_t  equipment_addres_3;            //参数地址
extern uint16_t  equipment_parament_3;

extern uint16_t  equipment_addres_4;            //参数地址
extern uint16_t  equipment_parament_4;

extern uint16_t  equipment_addres_5;            //参数地址
extern uint16_t  equipment_parament_5;

extern uint16_t  equipment_addres_6;          //参数地址
extern uint16_t  equipment_parament_6;

extern uint16_t  equipment_addres_7;
extern uint16_t  equipment_parament_7;

extern uint16_t  equipment_addres_8;
extern uint16_t  equipment_parament_8;

extern uint16_t  equipment_addres_9;
extern uint16_t  equipment_parament_9;

extern uint16_t  equipment_addres_1_1;             //第一组：参数地址
extern uint16_t  equipment_parament_1_1;                      //出仓复位补偿



extern uint16_t    Unit1_Status_addres; 

extern uint16_t  	Unit2_Status_addres;

extern uint16_t    Unit3_Status_addres;





void Read_equipment_parament(void);

void I2C_write2Byte(uint16_t equipment_addres, uint16_t num);
uint16_t I2C_get2Byte(uint16_t equipment_addres);

void I2C_writeByte(uint16_t equipment_addres, uint8_t num);
uint8_t I2C_getByte(uint16_t equipment_addres);

/************************************************************************/
#endif


