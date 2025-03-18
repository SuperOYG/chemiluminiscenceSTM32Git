#include "motor.h"
#include "main.h"
#include "eeprom.h"
#include "Stdio.h"
#include "S_Moter1.h"
#include "S_Moter2.h"
#include "S_Moter3.h"


extern volatile uint32_t time;

uint32_t x_now; //ת�̵���ƶ�λ��
uint32_t y_now; //���Ե���ƶ�λ��
uint32_t z_now; //�˿�����ƶ�λ��


uint8_t  Channel_now; //��ǰ��ͷ�����ڵĲ��Կ�λ;

u8 MotorX_speed = 60;  //���Ʒ������ٶ�
u8 MotorY_speed = 15;  //���Է������ٶ�
u8 MotorZ_speed = 20;  //ж���������ٶ�

u8 speedgear=4;//�ٶȵ�λ
extern u8 NumberOfCards;

extern u8 testStatus;

//��ǰ���Ե��Լ��Ŀ������
extern u8 testIndex;


extern u8 StartStep;

extern u8 SingleStep;


extern u32 moter3_now;      //Z������ǰλ��
extern u32 moter3_next;     //Z������һ��λ��

extern u8 motor3Flag; //  Z������λ��־λ

extern u32 moter1_now;      //x������ǰλ��
extern u32 moter1_next;     //x������һ��λ��

extern u32 moter2_now;      //y������ǰλ��
extern u32 moter2_next;     //y������һ��λ��

extern u8 motor1Flag; //  x������λ��־λ

void Delay_moter(uint16_t n)
{
    unsigned int i;
    for(; n>0; n--) {
        for(i=50; i>0; i--);
    }
}

/******************************************************/

void Ka1_moter(void)
{
    CP1_TOGGLE;
//	CP1_TOGGLE;
}

void Ka2_moter(void)
{
    CP2_TOGGLE;
}

void Ka3_moter(void)
{
    CP3_TOGGLE;
}


/**
*   @brief: ��ⷽ������λ  ��·��������1�����
*   @param:  delay���ٶȣ�direction ��Ϊ0 ���⸴λ�� Ϊ1���ڲิλ��
*   @return: void
*   @author Hang
*   @date: 2023-03-30
*/
void Ka1_InitBack(u8 delay,u8 direction)
{
    u16 step = 200;
    u32 stop_flag = 0;
//	u8 cnt=1;
    Delay_moter(1000);
    if(direction==1) {
//	EN1_H;
        EN1_L;
        DIR1_H;
//	GENERAL_TIM_Init();							/*��ʱ����ʼ��*/
//	TIM_Cmd(GENERAL_TIM, DISABLE);				//�رն�ʱ��
//	TIM_Cmd(GENERAL_TIM, ENABLE);				//������ʱ��
        time = 0;
//	while(HAL_GPIO_ReadPin(IN1_GPIO_Port,IN1_Pin) == KEY_ON)   //PC7     P7�ӿ�
        while(HAL_GPIO_ReadPin(IN1_GPIO_Port,IN1_Pin) == KEY_ON)   //    P13�ӿ�
        {
            if(time == delay*15)
            {   time = 0;
                Ka1_moter();
                stop_flag++;
                if(stop_flag == 500000)
                {
                    EN1_L;
                }
//					if(step==400){
//						delay=delay*speedgear;
//					}
            }
        }
        DIR1_L;
        while(step) {
            Delay_moter(100);
            Ka1_moter();
            step--;
        }
        y_now = 0;  //λ���������
        EN1_L;
    }
    else {
//	EN1_H;
        EN1_L;
        DIR1_L;
//	GENERAL_TIM_Init();							/*��ʱ����ʼ��*/
//	TIM_Cmd(GENERAL_TIM, DISABLE);				//�رն�ʱ��
//	TIM_Cmd(GENERAL_TIM, ENABLE);				//������ʱ��
        time = 0;
        while(HAL_GPIO_ReadPin(IN1_GPIO_Port,IN1_Pin) == KEY_ON)   //PC7     P7�ӿ�
//	while(HAL_GPIO_ReadPin(IN5_GPIO_Port,IN5_Pin) == KEY_ON)   //    P13�ӿ�
        {
            if(time == delay*15)
            {   time = 0;
                Ka1_moter();
                stop_flag++;
                if(stop_flag == 500000)
                {
                    EN1_L;
                }
//					if(step==400){
//						delay=delay*speedgear;
//					}
            }
        }
        DIR1_H;
        while(step) {
            Delay_moter(100);
            Ka1_moter();
            step--;
        }
        y_now = 0;  //λ���������
        EN1_L;
    }
}



/**
*   @brief: ת�̸�λ  �����ڶ������
*   @param:  void
*   @return: void
*   @author Hang
*   @date:  2023-03-30
*/
void Ka2_InitBack(u8 delay)
{
    u16 step1 = 1334; //������λ���غ�������˾��롣
    u32 stop_flag = 0;
    Delay_moter(1000);
    EN2_H;
    DIR2_H;
//	GENERAL_TIM_Init();						/*��ʱ����ʼ��*/
//	TIM_Cmd(GENERAL_TIM, DISABLE);				//�رն�ʱ��
//	TIM_Cmd(GENERAL_TIM, ENABLE);				//������ʱ��
    time = 0;
    while(HAL_GPIO_ReadPin(IN1_GPIO_Port,IN1_Pin) == KEY_OFF)    // ����5  ��P8�ӿ�
    {
        if(time == delay*60)
        {
            time = 0;
            Ka2_moter();
            stop_flag++;
            if(stop_flag == 500000)
            {
                EN2_L;
            }
        }
    }
    DIR2_H;
    while(step1)
    {
        Delay_moter(100);
        Ka2_moter();
        step1--;
    }
    x_now = 0;
//		EN1_L;
}


/**
*   @brief: ת�̸�λ���ƶ�����  �����ڶ������
*   @param:  void
*   @return: void
*   @author Hang
*   @date:  2023-03-30
*/
void Ka2_InitBack2(u8 delay,u16 distance)
{
    u16 step1 = distance; //������λ���غ�����ƶ����롣
    u32 stop_flag = 0;
    Delay_moter(1000);
    EN2_H;
    DIR2_H;
    u32 totalStep;
//	GENERAL_TIM_Init();						/*��ʱ����ʼ��*/
//	TIM_Cmd(GENERAL_TIM, DISABLE);				//�رն�ʱ��
//	TIM_Cmd(GENERAL_TIM, ENABLE);				//������ʱ��
    time = 0;
    while(HAL_GPIO_ReadPin(IN1_GPIO_Port,IN1_Pin) == KEY_OFF)    // ����5  ��P8�ӿ�
    {
        if(time == delay*45)
        {
            time = 0;
            Ka2_moter();
            stop_flag++;
            if(stop_flag == 500000)
            {
                EN2_L;
            }
//				totalStep++;
        }
    }

    DIR2_H;
    time = 0;
    while(step1)
    {
//			Delay_moter(100);Ka2_moter();step1--;
        if(time == delay*45)
        {
            time = 0;
            Ka2_moter();
            stop_flag++;
            if(stop_flag == 500000)
            {
                EN2_L;
            }
            step1--;
        }
    }
    x_now = 0;
//    	EN2_L;
}



/**
*   @brief: �˿������λ  ������3�����
*   @param:  void
*   @return: void
*   @author Hang
*   @date: 2023-3-30
*/
void Ka3_InitBack(u8 delay) {
    u16 step = 700;
    u32 stop_flag = 0;
    u8 cnt=1;
    Delay_moter(1000);
    EN3_H;
    DIR3_L;
//	GENERAL_TIM_Init();							/*��ʱ����ʼ��*/
//	TIM_Cmd(GENERAL_TIM, DISABLE);				//�رն�ʱ��
//	TIM_Cmd(GENERAL_TIM, ENABLE);				//������ʱ��
    time = 0;
    while(HAL_GPIO_ReadPin(IN1_GPIO_Port,IN1_Pin) == KEY_ON)          //PB13    P12�ӿ�
    {
        if(time == delay*5)
        {   time = 0;
            Ka3_moter();
            stop_flag++;
            if(stop_flag == 500000)
            {
                EN3_L;
            }
        }
    }
    DIR3_H;
    while(step)
    {
        Delay_moter(30);
        Ka3_moter();
        step--;
    }
    z_now = 0;
    EN3_L;
}


/**
//------------------CurUser--------------
*   @brief: ���Է���ĵ���ƶ�����  ��������Ϊ����ԭ�㡣
*   @param:  delay:��������
			 distance:λ�ƾ���
*   @return: void
*   @author Hang
*   @date: 2023-03-30
*/
void Ka1Moter_Step(u8 delay,u32 distance)
{
    u16 step;
//	EN1_H;
    EN1_L;
//	if(distance>10000) distance=10000;
    printf("y���ƶ�����%d",distance);
    u32 Ka_next = distance*2;
    if(Ka_next>y_now) {
        step = Ka_next-y_now;    //
        DIR1_H;
    }
    else if(Ka_next<y_now) {
        step = y_now - Ka_next;
        DIR1_L;
    }
    else step = 0;
//	GENERAL_TIM_Init();			/*��ʱ����ʼ��*/
//	TIM_Cmd(GENERAL_TIM, DISABLE);	//�رն�ʱ��
//	TIM_Cmd(GENERAL_TIM, ENABLE);		//������ʱ��
    time = 0;
    while(step)
    {
        if(time == delay*MotorY_speed)
        {
            time = 0;
            Ka1_moter();
            step--;
        }
    }
    EN1_L;
//	EN1_H;
    y_now = Ka_next;
}

/**
// ---------------CurUse-------------------
*   @brief: ת��λ�ƿ���
*   @param:  delay:��������
			 distance:λ�ƾ���
*   @return: void
*   @author Hang
*   @date: 2023-3-30
*/
void Ka2Moter_Step(u8 delay,u32 distance)
{
    u16 step;
//	if(distance>14000) distance=14000;
    u32 Ka_next = distance;
//	if(y_now!=0){      // ����֮ǰһ���жϲ��Է����Ƿ����
////		Ka3_InitBack(1);   // �˿������λ
////		HAL_Delay(200);
////		Ka1_InitBack(1,1);   // ���Ե����λ
////		HAL_Delay(200);
//	}
    EN2_H;
    if(Ka_next>x_now) {
        step = Ka_next-x_now;
        DIR2_H;
    }
    else if(Ka_next<x_now) {
        step = x_now - Ka_next;    //��������������ת��ֻ��˳ת������ֻ�������á�
        DIR2_L;
    }
    else step = 0;
//	GENERAL_TIM_Init();			/*��ʱ����ʼ��*/
//	TIM_Cmd(GENERAL_TIM, DISABLE);	//�رն�ʱ��
//	TIM_Cmd(GENERAL_TIM, ENABLE);		//������ʱ��
    time = 0;
    while(step)
    {
        if(time == delay*MotorX_speed)
        {
            time = 0;
            Ka2_moter();
            step--;
        }
    }
    x_now = Ka_next;
    printf("ת�̵�ǰλ��Ϊ%d",x_now);
//	EN2_L;
}
/**
*   @brief: ж������ƶ�
*   @param:  void
*   @return: void
*   @author Hang
*   @date: 2023-3-30
*/

void Ka3Moter_Step(u8 delay,u32 distance)
{
    u16 step;
    EN3_H;
////	if(distance>10000) distance=10000;
    printf("Z���ƶ�����%d",distance);
    u32 Ka_next = distance;
    if(Ka_next>z_now) {
        step = Ka_next-z_now;    //
        DIR3_H;
    }
    else if(Ka_next<z_now) {
        step = z_now - Ka_next;
        DIR3_L;
    }
    else step = 0;
//	GENERAL_TIM_Init();			/*��ʱ����ʼ��*/
//	TIM_Cmd(GENERAL_TIM, DISABLE);	//�رն�ʱ��
//	TIM_Cmd(GENERAL_TIM, ENABLE);		//������ʱ��
    time = 0;
    while(step)
    {
        if(time == delay*4)
        {
            time = 0;
            Ka3_moter();
            step--;
        }
    }
    EN3_L;
    z_now = Ka_next;
}


/**
*   @brief: �ӿ�����
*   @param:  void
*   @return: void
*   @author Hang
*   @date: 2023-3-30
*/
/***************   ���Ӽ��ٿ��Ƶ��1**************/
void moter1_step(u32 distance,u8 flag)//distance = ����;
{
//	motor1Flag=0;
    u32 step;
    moter1_next = distance;
    if(moter1_next > moter1_now)
    {
        DIR1_H;
        step = moter1_next - moter1_now;
    }
    else if(moter1_next < moter1_now)
    {
        DIR1_L;
        step = moter1_now - moter1_next;
    }
    else
    {
        step = 0;
    }
//    printf("step �Ĵ�СΪ��%d",step);
    if(step > 500)
    {
        Moter1_Run(flag,1,step);
    }
    if(step == 500)
    {
        Moter1_Run(flag,1,step);
    }
    if((step > 0)&&(step < 500))
    {
        Moter1_Run(flag,2,step);
    }
    if(step == 0)
    {;}
    moter1_now = moter1_next;
}

void moter1_int(u8 step) {
    DIR1_L;
    Moter1_Run(1,3,7000);  //10000��Ϊ�˶���Զ���Ĳ�����������
    DIR1_H;
    Moter1_Run(1,2,step); //�������أ��ز�һС����
    moter1_now=0;
}

/***************   ���Ӽ��ٿ��Ƶ��2**************/
void moter2_step(u32 distance,u8 flag)//distance = ����;
{
    u32 step;
    if(distance>3000) { //�����ƶ������ˡ�
        return;
    }

    moter2_next = distance;

    if(moter2_next > moter2_now)
    {
        DIR2_H;
        step = moter2_next - moter2_now;
    }
    else if(moter2_next < moter2_now)
    {
        DIR2_L;
        step = moter2_now - moter2_next;
    }
    else
    {
        step = 0;
    }
//    printf("step �Ĵ�СΪ��%d",step);
    if(step > 500)
    {
        Moter2_Run(flag,1,step);
    }
    else if(step == 500)
    {
        Moter2_Run(flag,1,step);
    }
    else if((step > 0)&&(step < 500))
    {
        Moter2_Run(flag,2,step);
    }
    moter2_now = moter2_next;
}

void moter2_int(u8 step) {
    DIR2_L;
    Moter2_Run(1,2,5000);  //10000��Ϊ�˶���Զ���Ĳ�����������
    DIR2_H;
    Moter2_Run(1,2,step); //�������أ��ز�һС����
    moter2_now=0;
}



