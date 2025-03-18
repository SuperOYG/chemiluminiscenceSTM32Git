#include "motor.h"
#include "main.h"
#include "eeprom.h"
#include "Stdio.h"
#include "S_Moter1.h"
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
extern u32 moter3_next;     //Z������ǰλ��

extern u8 motor3Flag; //  Z������λ��־λ

extern u32 moter1_now;      //x������ǰλ��
extern u32 moter1_next;     //x������ǰλ��

extern u8 motor1Flag; //  x������λ��־λ

void Delay_moter(uint16_t n)
{
	unsigned int i;
	for(;n>0;n--){for(i=50;i>0;i--);}
}

/******************************************************/

void Ka1_moter(void)      
{  
	CP1_TOGGLE;
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
	if(direction==1){
	EN1_H;
	DIR1_H;
//	GENERAL_TIM_Init();							/*��ʱ����ʼ��*/
//	TIM_Cmd(GENERAL_TIM, DISABLE);				//�رն�ʱ��
//	TIM_Cmd(GENERAL_TIM, ENABLE);				//������ʱ��	
	time = 0;
//	while(HAL_GPIO_ReadPin(IN1_GPIO_Port,IN1_Pin) == KEY_ON)   //PC7     P7�ӿ�
	while(HAL_GPIO_ReadPin(IN5_GPIO_Port,IN5_Pin) == KEY_ON)   //    P13�ӿ�
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
		while(step){
			Delay_moter(100);Ka1_moter();step--;
		}
		y_now = 0;  //λ���������
		EN1_L;
	}
	else {
	EN1_H;
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
		while(step){
			Delay_moter(100);Ka1_moter();step--;
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
	while(HAL_GPIO_ReadPin(IN6_GPIO_Port,IN6_Pin) == KEY_OFF)    // ����5  ��P8�ӿ�
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
			Delay_moter(100);Ka2_moter();step1--;
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
	while(HAL_GPIO_ReadPin(IN6_GPIO_Port,IN6_Pin) == KEY_OFF)    // ����5  ��P8�ӿ�
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
void Ka3_InitBack(u8 delay){
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
	while(HAL_GPIO_ReadPin(IN4_GPIO_Port,IN4_Pin) == KEY_ON)          //PB13    P12�ӿ� 
	  {
			if(time == delay*5)
			{time = 0;
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
			Delay_moter(30);Ka3_moter();step--;
		}
		z_now = 0;
		EN3_L;	
}



/**
*   @brief: �����ʼ�������λ�ù���
*   @param:  void
*   @return: void
*   @author Hang
*   @date: 
*/
void Motor_init()
{
  //M1_ON;M2_ON;M3_OFF;  //110  16ϸ��
	REF1_H;
	REF2_H;
	REF3_H;
    EN1_H;
	EN2_H;
	EN3_H;
	moter3_int();//ж����λ�����Ӽ��٣� 
//	Ka3_InitBack(1);   // �˿������λ
//	HAL_Delay(1000);
	Ka1_InitBack(1,1);   // ���Ե����λ
//	HAL_Delay(500);
//	Ka2_InitBack(1);   //  ת�̸�λ
	Ka2_InitBack2(1,1280);
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
	EN1_H;
//	if(distance>10000) distance=10000;
	printf("y���ƶ�����%d",distance);
	u32 Ka_next = distance;
	if(Ka_next>y_now){step = Ka_next-y_now;DIR1_H;}    //
	else if(Ka_next<y_now) {step = y_now - Ka_next;DIR1_L;}
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
	if(Ka_next>x_now){step = Ka_next-x_now;DIR2_H;}
	else if(Ka_next<x_now) {step = x_now - Ka_next;DIR2_L;}  //��������������ת��ֻ��˳ת������ֻ�������á�
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
	if(Ka_next>z_now){step = Ka_next-z_now;DIR3_H;}    //
	else if(Ka_next<z_now) {step = z_now - Ka_next;DIR3_L;}
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

void addCard(){
	printf("����忨�ȴ�ģʽ");
	while(NumberOfCards<20&&testStatus==0){
		HAL_Delay(2000);
		if(HAL_GPIO_ReadPin(IN2__GPIO_Port,IN2__Pin) == KEY_OFF
			&&HAL_GPIO_ReadPin(IN1__GPIO_Port,IN1__Pin) == KEY_OFF){
			NumberOfCards++;
//			Ka2_InitBack2(1,480*NumberOfCards+1280);
			Ka2Moter_Step(1,480*NumberOfCards); //ת��ת��һ��
			printf("��ǰ�п�����");
		}else{
			printf("��ʱ�޿�����");
		}
		printf("testStatus=%d",testStatus);
	}
}

//u8 checkCard(){

//	if(){
//	}




//}
/**
*   @brief: ж���������Ӳ���ɨ����ɵ㴦���Լ���ת�Ƶ�ж�ؿ�ж�ء�
*   @param:  void
*   @return: void
*   @author Hang
*   @date: 2023-3-30
*/
void dropCard(){
	Ka1Moter_Step(1,0);   //���Ե������ิλ
	HAL_Delay(500);
	Ka2Moter_Step(1,2400); // ת���ƶ�
	HAL_Delay(500);
	moter3_step(35000);// ж���ƶ� 
	HAL_Delay(500);
	moter3_int();//ж����λ
	HAL_Delay(500);
	Ka2_InitBack2(1,1280);// ת�̸�λ
	NumberOfCards--;
}


/**
*   @brief: �⿨�Ͽ�����
*   @param:  testId : ��λ��ָ����ǰ���ԵĿ������
*   @return: void
*   @author Hang
*   @date: 2023-3-30
*/

void testCard(u8 testId){
	testIndex=testId;
		if(NumberOfCards!=0){
	// ����һ���Լ������ƶ������λ
	u8 moveStep=(testIndex-1)*equipment_parament_1+equipment_parament_2;		
	Ka2Moter_Step(1,moveStep);   	
	}  
	Ka1Moter_Step(1,equipment_parament_4);   //�ƶ�������λ�á�
}



void Delay_moter3(u16 n)
{
	unsigned int i;
	for(;n>0;n--){for(i=100;i>0;i--);}
}

/***************   ���Ӽ��ٿ��Ƶ��3***************/
void moter3_1step(u8 speed)      
{  
    Delay_moter3(speed);
	CP3_TOGGLE;
}

void moter3_step(u32 distance)//distance = ����;
{         
	motor3Flag=0;
	u32 step;
	moter3_next = distance;
  
  if(moter3_next > moter3_now)       
    {
      DIR3_H;
      step = moter3_next - moter3_now;
    }
  else if(moter3_next < moter3_now)  
    {
      DIR3_L;
      step = moter3_now - moter3_next;
    }
  else                                 
    {step = 0;}
 printf("step �Ĵ�СΪ��%d",step);
  if(step > 500)							
		{Moter3_Run(1,0,step);}
	if(step == 500)							
		{Moter3_Run(1,1,step);}
	if((step > 0)&&(step < 500))
		{Moter3_Run(1,2,step);}
	if(step == 0)								
		{;}
  moter3_now = moter3_next; 
  //1.����ǰZ����д��eeprom;
  I2C_write2Byte(equipment_addres_7, moter3_now);				
}


//���3 ��λ��
void moter3_int()
{   	
	motor3Flag=1;
	u32 step;
      DIR3_L;
 step=I2C_get2Byte(equipment_addres_7);
 printf("moter3_now �Ĵ�СΪ��%d",moter3_now);	
 printf("step �Ĵ�СΪ��%d",step);
  if(step > 500)							
		{Moter3_Run(0,0,step);}
	if(step == 500)							
		{Moter3_Run(0,1,step);}
	if((step > 0)&&(step < 500))
		{Moter3_Run(0,2,step);}
	if(step == 0)								
		{;}
  moter3_now = 0;
  I2C_write2Byte(equipment_addres_7, moter3_now);				
}



/***************   ���Ӽ��ٿ��Ƶ��1**************/
void moter1_1step(u8 speed)      
{  
    Delay_moter3(speed);
	CP1_TOGGLE;
}

void moter1_step(u32 distance)//distance = ����;
{         
//	motor1Flag=0;
	u32 step;
	moter1_next = distance;
  
  if(moter1_next > moter1_now)       
    {
      DIR3_H;
      step = moter1_next - moter1_now;
    }
  else if(moter1_next < moter1_now)  
    {
      DIR3_L;
      step = moter1_now - moter1_next;
    }
  else                                 
    {step = 0;}
 printf("step �Ĵ�СΪ��%d",step);
  if(step > 500)							
		{Moter1_Run(1,0,step);}
	if(step == 500)							
		{Moter1_Run(1,1,step);}
	if((step > 0)&&(step < 500))
		{Moter1_Run(1,2,step);}
	if(step == 0)								
		{;}
  moter1_now = moter1_next; 
//  //1.����ǰZ����д��eeprom;
//  I2C_write2Byte(equipment_addres_7, moter1_now);				
}


//���1 ��λ��
void moter1_int()
{   	
	motor3Flag=1;
	u32 step;
      DIR3_L;
 step=I2C_get2Byte(equipment_addres_7);
 printf("moter1_now �Ĵ�СΪ��%d",moter1_now);	
 printf("step �Ĵ�СΪ��%d",step);
  if(step > 500)							
		{Moter1_Run(0,0,step);}
	if(step == 500)							
		{Moter1_Run(0,1,step);}
	if((step > 0)&&(step < 500))
		{Moter1_Run(0,2,step);}
	if(step == 0)								
		{;}
  moter1_now = 0;
  I2C_write2Byte(equipment_addres_7, moter1_now);				
}








////ɨ��ʹ�õ�Y���ƶ�������
//void Ka1Moter_RunStep(u8 delay,u32 distance)
//{  
//	u16 step;
////	EN1_H;
////	if(distance>11000) distance=11000;
//	u32 Ka_next = distance;
//	if(Ka_next>y_now){step = Ka_next-y_now;DIR1_L;}
//	else if(Ka_next<y_now) {step = y_now - Ka_next;DIR1_H;}
//	else step = 0;
////	GENERAL_TIM_Init();			/*��ʱ����ʼ��*/
////	TIM_Cmd(GENERAL_TIM, DISABLE);	//�رն�ʱ��
////	TIM_Cmd(GENERAL_TIM, ENABLE);		//������ʱ��
//	time = 0;
//	while(step)
//	{
//		if(time == 20)
//			{
//				time = 0;
//				Ka1_moter();
//				step--;
//			}
//	}
////	EN1_L;
//	y_now = Ka_next;
//}


///**
//*   @brief����ѹɨ�������ɺ󣬵��ж���Լ��������
//*   @param:  
//*   @return: 
//*   @author Hang
//*   @date: 2023-01-15
//*/
//void Moter_DropCard(){
////1.���Է������λ
//	Ka1_InitBack(MotorY_speed);
////2.���Ʒ�������ƫ��ж��λ��
//	Ka2Moter_Step(1,x_now-equipment_parament_3);
////3.���Է�������������ˡ�
//	Ka1Moter_Step(1,equipment_parament_4);
////4.���Է������λ��
//	Ka1_InitBack(MotorY_speed);
////5.���Ʒ���ص�ԭ�㡣	
//	Ka2_InitBack(MotorX_speed);
//}

///**
//*   @brief: �ƶ���ָ��ͨ����ָ��λ��
//*   @param:  n:��ǰҪ���ԵĿ�λ flag-> 0��ɨ����λ�� 1:��ѹɨ��λ�� 
//*   @return: void
//*   @author Hang
//*   @date: 
//*/
//void Moter_MoveToChannel(u8 n,u8 flag){
//	if(n!=Channel_now){
//		if(n>6) n=6;
//	
//	int move_ToXStep=equipment_parament_2+(n-1)*equipment_parament_1; //���ƾ���
//	if(n>3) move_ToXStep=move_ToXStep+(n-3)*40; //����ƫ����,ʹ��ͷ��Կ����ġ�
//		Ka2Moter_Step(01,move_ToXStep);   //���Ʒ���
//	}
//	if(flag==0){
//		int moveToYStep=equipment_parament_5;  //���Է���
//		Ka1Moter_Step(01,moveToYStep);
//	}else if(flag==1){  
//		int moveToYStep=equipment_parament_6;  
//		Ka1Moter_Step(01,moveToYStep);	//���Է���	
//	}
//	Channel_now=n;
//}

