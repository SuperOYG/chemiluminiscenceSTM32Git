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



//��ҳ��ַ��Χ��0x00~0x1F  ��������ز�����   �Ի���������һ��Ϊ��һ��
//-----------------------------�����λ��������----------------------------------
uint16_t  equipment_addres_1 = 0x0000;               // ��һ�飺������ַ
uint16_t  equipment_parament_1;                      //���ָ�λ����

uint16_t  equipment_addres_2 = 0x0002;               //������ַ
uint16_t  equipment_parament_2;                      //�Ű���λ����

uint16_t  equipment_addres_3 = 0x0004;               //������ַ
uint16_t  equipment_parament_3;                      //�𵴸�λ����

uint16_t  equipment_addres_4 = 0x0006;               //�ڶ��飺������ַ
uint16_t  equipment_parament_4;                      //���ָ�λ����

uint16_t  equipment_addres_5 = 0x0008;               //������ַ
uint16_t  equipment_parament_5;                      //�Ű���λ����

uint16_t  equipment_addres_6 = 0x000A;               //������ַ��
uint16_t  equipment_parament_6;                      // �𵴸�λ����

uint16_t  equipment_addres_7 = 0x000C;               //������ ������ַ��
uint16_t  equipment_parament_7;                      //���ָ�λ����

uint16_t  equipment_addres_8 = 0x000E;               //������ַ��
uint16_t  equipment_parament_8;                      //�Ű���λ����

uint16_t  equipment_addres_9 = 0x0010;               //������ַ��
uint16_t  equipment_parament_9;    					 //�𵴸�λ����
//-- �������������Ӧ�����Ĳ��������ţ����һ�����ֲ������������׵�ַ����������ĵ�ַ��Ӧ��1 ����--
uint16_t  equipment_addres_1_1 = 0x0012;             //��һ�飺������ַ
uint16_t  equipment_parament_1_1;                      //���ָ�λ����






extern uint16_t  equipment_addres_1_1;             //��һ�飺������ַ
extern uint16_t  equipment_parament_1_1;                      //���ָ�λ����

extern uint16_t  equipment_addres_1_2;             //������ַ
extern uint16_t  equipment_parament_1_2;                      //�Ű���λ����

extern uint16_t  equipment_addres_1_3;               //������ַ
extern uint16_t  equipment_parament_1_3;                      //�𵴸�λ����

extern uint16_t  equipment_addres_1_4;               //�ڶ��飺������ַ
extern uint16_t  equipment_parament_1_4;                      //���ָ�λ����

extern uint16_t  equipment_addres_1_5;               //������ַ
extern uint16_t  equipment_parament_1_5;                      //�Ű���λ����

extern uint16_t  equipment_addres_1_6;               //������ַ��
extern uint16_t  equipment_parament_1_6;                      // �𵴸�λ����

extern uint16_t  equipment_addres_1_7;               //������ ������ַ��
extern uint16_t  equipment_parament_1_7;                      //���ָ�λ����

extern uint16_t  equipment_addres_1_8;               //������ַ��
extern uint16_t  equipment_parament_1_8;                      //�Ű���λ����

extern uint16_t  equipment_addres_1_9;               //������ַ��
extern uint16_t  equipment_parament_1_9;    					 //�𵴸�λ����



//��ҳ��ַ��Χ��0x20~0x1F  ���浥Ԫ���״̬������ ���Ǵ��ڴ��ơ���Һ����״̬��

uint16_t    Unit1_Status_addres = 0x0020; 

uint16_t  	Unit2_Status_addres = 0x0021;

uint16_t    Unit3_Status_addres = 0x0020;




void convertNumberToArray(uint32_t number, uint8_t* array);

//-----------------------------�����λ��������----------------------------------


void I2CTest(void);

//�ӵ�ǰ��ַ�����ȡ�����ֽ�����
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

//��ȡ��ǰ��ַλ������ֵ
uint8_t I2C_getByte(uint16_t equipment_addres)
{
    uint8_t readBuffer[1] = {0};
    uint16_t a;
    HAL_I2C_Mem_Read(&hi2c1, ADDR_24LCxx_Read, equipment_addres, I2C_MEMADD_SIZE_16BIT, readBuffer, 1, 1000);
    a = readBuffer[0];
    printf("para1=0x%02X\r\n", readBuffer[0]);
    return a;
}

//����ǰ��ַλд��һ���ֽ�����
void I2C_writeByte(uint16_t equipment_addres, uint8_t num)
{
    uint8_t writerbuf[1] = {num};
    HAL_I2C_Mem_Write(&hi2c1, ADDR_24LCxx_Write, equipment_addres, I2C_MEMADD_SIZE_16BIT, writerbuf, 1, 100);
}

//����ǰ��ַλ��ʼд��2���ֽ�����
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

//���͵����λУ׼����
void sendMotorParams(u16 address,u8 boardCode,u8 moterCode) {
    uint8_t TextBuffer[8];
    uint8_t hexArray[5];
    u8 symbolAddress=address/2+equipment_addres_1_1; //���������ŵ�ַλ��
    TextBuffer[0]=0x04; //��־λ
//    printf("��ǰ��ȡ���������ŵĵ�ַλ��%d\r\n",symbolAddress);
    TextBuffer[3]=I2C_getByte(symbolAddress);//������
    equipment_parament_1 = I2C_get2Byte(address);
//    printf("��ǰ��ȡ�����ĵ�ַλ%d\r\n",address);
//    printf("��һ����ֵ������=%d\r\n",equipment_parament_1);
    convertNumberToArray(equipment_parament_1,hexArray);
    for (int i = 0; i < 4; i++)  //ֻȡת��������ĺ���λ��
    {
        TextBuffer[i+4] = hexArray[i+1];
    }
    TextBuffer[1]=boardCode;//��ţ�
    TextBuffer[2]=moterCode;//�����:
    CAN_senddata(&hcan, TextBuffer, boardCode);//���͸�1���Ӱ�
    HAL_Delay(100);//���ͼ��ʱ�䣺
	
    for(int i=0; i<8; i++) { 
        Usart2_SendByte(TextBuffer[i]);
    }
	//��ʱ�ȴ���ǰ����������ɡ�
	
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

//��ȡ������I2C�ڵ�ϵͳ������
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
//I2C ����---

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
*   @brief:  ��0~99999 ֮���������ֵת����[3x,3x,3x,3x,3x]����ʽ��
*   @param:  void
*   @return: void
*   @author Hang
*   @date: 2023-05-28
*/

void convertNumberToArray(uint32_t number, uint8_t* array) {
    array[0] = (number / 10000) + 0x30;  // ��λ
    array[1] = ((number / 1000) % 10) + 0x30;  // ǧλ
    array[2] = ((number / 100) % 10) + 0x30;  // ��λ
    array[3] = ((number / 10) % 10) + 0x30;  // ʮλ
    array[4] = (number % 10) + 0x30;  // ��λ
}












