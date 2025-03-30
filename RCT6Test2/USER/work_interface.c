#include "work_interface.h"
#include "eeprom.h"

#include "can.h"
#include "i2c.h"
#include "gpio.h"
#include "motor.h"
#include "math.h"
#include "tim.h"
#include "S_Moter1.h"
#include "S_Moter2.h"
#include "S_Moter3.h"
#include "bsp_stepper_S_speed.h"
//#include "PID_Realize.h"
#include "usart.h"
#include "queue.h"


extern UART_HandleTypeDef huart1;
extern Frame currentFrame;
extern MessageQueue msgQueue;  // ȫ����Ϣ����

//�жϼ�ʱ����
uint32_t time;
uint8_t R0, R1, R2, R3, R4, R5, R6, R7, R8, R9; //�������մ����·���ָ�

//��ǰʹ�õĴ���λ��
uint8_t CurUartNum;

uint8_t  RecieveBuffer[1];//�ݴ���յ����ַ�
uint8_t  Rx_end;  //ָ�������ɱ�־λ
uint8_t  RxLine;  //rxbuf���յ����ݳ���Ϊ:RxLine+1;
uint8_t  rxbuf[50];//�յ������ݴ�Ŵ�
uint8_t TextBuffer[8] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x44, 0x55, 0x55};

//�𵴵���ƶ�
int MoveStep;

/**
******���ʹ�õ��м����*********
*/
//����һ��������ƶ�λ�����飺
int slipAddress[15] = {17760, 16358, 0, 0, 0, 0, 0, 0, 0, 0, 8312, 7290, 6385, 5480, 4620};
/*
*  *********************************���ʹ�õ��м����*******************************
*/
u32 moter3_now;      //Z������ǰλ��
u32 moter3_next;     //Z������һ��λ��

u32 moter1_now;      //X������ǰλ��
u32 moter1_next;     //X������һ��λ��

u32 moter2_now;      //y������ǰλ��
u32 moter2_next;     //y������һ��λ��

u8 motor1Flag;//���1��λ��־λ��
u8 motor2Flag;//���2��λ��־λ��
u8 motor3Flag;//���2��λ��־λ:

//���ֵ����λƫ�����
int motor1_ResetParams=200; //2025��3��24��20:50:37
//�Ű������λƫ�����
int motor2_ResetParams;
//�𵴵����λƫ�����
int motor3_ResetParams;

//����ƶ��ٶȣ�
u16 motor1_Speed;
u16 motor2_Speed;
u16 motor3_Speed = 400;

//��ǰ����״̬
uint8_t testStatus;

u8 moter2_RestNum; //�𵴵����λ������

//������3������������ѭ�����ԣ�
u8  testTimes;//ѭ�����Դ�����
u32 moter3_position;//�𵴵���ƶ����룻
u32 heighOfLiquid;//��Һ���룻

u8  channel1OfLiquid; //ȡҺ��λ
u8  channel2OfLiquid; //��Һ��λ

//������3���������������𵴲��ԣ�
u8 timesOfShocck;//�𵴴���
u32 height1OfShock; //��λ��
u32 height2OfShock;
//u32 speedOfShock = 200; //���ٶ�  ��λ�� r/min(��λʱ����ת��Ȧ��);
u32 speedOfShock = 400; //���ٶ�  ��λ�� r/min(��λʱ����ת��Ȧ��);


//��Һ��ʼ�߶�
u16 aspiration_height=8000;
//�л����ģʽ��
void ChangeGPIOMode(u8 flag, u8 moterId);

//�ظ���Һ������
void LoopPickUpTest(u32 LiquidHeight, u32 motor3_Position, u8 times);
//�𵴲��Ժ�����
void ShockTest(u32 height1, u32 height2, u32 times);
//��ʱms:
void ms_Delay(uint16_t t_ms);



//���ϸ�������޿���·��ĵ��ϸ����32���ҵĵ��1,2��4ϸ�֣� �𶯵����ϸ����8ϸ�֣�
//���ԣ���Ҫ����ԭ�г���Ļ����ϣ���һ���˶�������ϵ�������1,2 ��8 �����3��4ϸ�֣�
//uint8_t Motor1_2Factor=8;
//uint8_t Motor3Factor=4;


//���1 ��λ��
void moter1_int2(u16 CurPosition, u16 step)
{
    u16 moveStep = CurPosition + 400;
    DIR1_L;
    Moter1_Run(1, 0, moveStep); //10000��Ϊ�˶���Զ���Ĳ�����������
    DIR1_H;

    moter1_now = 0;
    HAL_Delay(200);
    Moter1_Run(0, 0, step); //�������أ��ز�һС����
    moter1_now = 0;
}

void moter1_move(u16 params)
{
    if (moter3_now != 0)
    {
        motor3Flag = 1;
        Stepper_Move_S(60, 200, 0.01f, -10000);
        while (Stepper.status != STOP)
        {
            HAL_Delay(10);
        }
        motor3Flag = 0;
        Stepper_Move_S(60, 200, 0.01f, -400); //�ز�һС����
        while (Stepper.status != STOP)
        {
            HAL_Delay(10);
        }
        moter3_now = 0;
    }
    params = params + motor1_ResetParams; //�����ֲ�������ӵ��ƶ�������
    moter1_step(params, 0);
}

void moter1_reset(u16 params)
{
    if (moter3_now != 0)
    {
        motor3Flag = 1;
        Stepper_Move_S(60, 200, 0.01f, -10000);
        while (Stepper.status != STOP)
        {
            HAL_Delay(10);
        }
        motor3Flag = 0;
        Stepper_Move_S(60, 200, 0.01f, -400); //�𵴵���ٴ������ƶ�200�����������滬����ײ��
        while (Stepper.status != STOP)
        {
            HAL_Delay(10);
        }
        moter3_now = 0;
    }
    if (params > 200)
    {
        moter1_int2(moter1_now, params);
    }
    else
    {
        moter1_int2(moter1_now, 200);
    }
}

void moter2_move(u16 params)
{
    params = params + motor2_ResetParams; //���Ű���������ӵ��ƶ�������
    moter2_step(params, 0);
}

void moter2_reset(u16 params)
{
    if (params > 200)
    {
        moter2_int(params);
    }
    else
    {
        moter2_int(200);
    }
}


void moter3_move(u16 params)
{
    params = params + motor3_ResetParams; //���𵴲�������ӵ��ƶ�������
    int moveStep = params - moter3_now;
    motor3Flag = 0;
    Stepper_Move_S(60, motor3_Speed, 0.01f, moveStep);
    while (Stepper.status != STOP)
    {
        HAL_Delay(10);
    }
    moter3_now = params;
}

void moter3_reset()
{
    if (moter3_now != 0)
    {
        motor3Flag = 1;
        Stepper_Move_S(60, 200, 0.01f, -10000);
        while (Stepper.status != STOP)
        {
            HAL_Delay(10);
        }
        motor3Flag = 0;
        Stepper_Move_S(60, 200, 0.01f, -400); //�ز�һС����
        while (Stepper.status != STOP)
        {
            HAL_Delay(10);
        }
        moter3_now = 0;
    }
}

/**********************************************************
 * @brief ��Ԫ��ĵ��3�Ḵλ
 * @author  hang
 * @date  
 **********************************************************/
void init()
{
    HAL_Delay(2000);
    moter2_int(200);//�Ű���λ
    if (HAL_GPIO_ReadPin(IN5_GPIO_Port, IN5_Pin) != 1) //�ж�һ���𵴵������ǰ������λ�ã�
    {
        motor3Flag = 1;
        Stepper_Move_S(60, 200, 0.01f, -10000);
        while (Stepper.status != STOP)
        {
            HAL_Delay(10);
        }
        motor3Flag = 0;         //�������غ���������400���� �����ƶ��Ű�ʱ����������ǹͷ��
        Stepper_Move_S(60, 200, 0.01f, -400);
        while (Stepper.status != STOP)
        {
            HAL_Delay(10);
        }
    }
    else
    {
        motor3Flag = 0;       
        Stepper_Move_S(60, 200, 0.01f, 1000);
        while (Stepper.status != STOP)
        {
            HAL_Delay(10);
        }
        motor3Flag = 1;
        Stepper_Move_S(60, 200, 0.01f, -10000);
        while (Stepper.status != STOP)
        {
            HAL_Delay(10);
        }
        motor3Flag = 0;         //�������غ���������400���� �����ƶ��Ű�ʱ����������ǹͷ��
        Stepper_Move_S(60, 200, 0.01f, -400);
        while (Stepper.status != STOP)
        {
            HAL_Delay(10);
        }
    }
    moter3_now = 0; //���ǰѴ��������غ���������ƶ�400��λ�ã���Ϊ�˶�����ԭ�㡣
    moter1_int(200);

	
	//��ʼ���������飺
					int n = (slipAddress[1] - slipAddress[10]) / 9;
					for (int i = 2; i < 10; i++)
					{
							slipAddress[i] = slipAddress[1] - n * (i - 1);
					}					
}

/**
*   @brief:  �ϴ�ʵʱ�¶�
*   @param:  void
*   @return: void
*   @author Hang
*   @date: 2023-04-11
*/
void uploadTemp()
{
  
}

/**
    *   @brief: �������Һ���ԣ�
    *   @param:  height: ��Һ�߶ȣ�channel1:ȡҺ��;channel����Һ�ۡ�
    *   @return: void
    *   @author Hang
    *   @date: 2023-05-24
    */
void aspirationTest(u16 height, u8 channel1, u8 channel2)
{
    u16 moter1_next;  //����
    u16 moter2_next; //�Ű�����
		aspiration_height=8000;//��Һ�Ű���ʼλ�� 
	//1.�Ű�������Һ��ʼ��
    moter2_move(aspiration_height); //�Ƶ���Һ��ʼλ�ã��ƶ���7000������ʼʵ�顣;2023-09-11  ����Ϊ6500 �������Ƿ����¸ɾ���
    //2.���ֵ�����Ƶ�ȡҺλ��
    moter1_next = slipAddress[channel1 - 1];
    moter1_move(moter1_next);
    //3.�𵴵��������ָ�����룺
    if (channel1 == 15) //����ȡ������
    {
		moter3_move(2450);
    }
    else      //�������ۣ�
    {
//		moter3_move(5400);//0918
		moter3_move(4400);//0918
    }
    //4.�Ű����ƣ���Һ
    moter2_next = aspiration_height - height;
    moter2_move(moter2_next);
    //5.�𵴵������λ
    moter3_reset();
//    //6.�Ű����ƣ�������
    moter2_next = moter2_next-200;
    moter2_move(moter2_next);
		moter1_reset(0);
}

/**********************************************************
 * @brief   ��ͣ��Һ
 * @param
 * @author  hang
 * @date  
 **********************************************************/
void aspirationTest_drop(){
		moter2_move(9000);
		HAL_Delay(2000);
    moter2_reset(0);
	 
}


//void aspirationTest(u16 height, u8 channel1, u8 channel2)
//{
//    u16 moter1_next;  //����
//    u16 moter2_next; //�Ű�����
//		aspiration_height=8000;//��Һ�Ű���ʼλ�� 
//	//1.�Ű�������Һ��ʼ��
//    moter2_move(aspiration_height); //�Ƶ���Һ��ʼλ�ã��ƶ���7000������ʼʵ�顣;2023-09-11  ����Ϊ6500 �������Ƿ����¸ɾ���
//    //2.���ֵ�����Ƶ�ȡҺλ��
//    moter1_next = slipAddress[channel1 - 1];
//    moter1_move(moter1_next);
//    //3.�𵴵��������ָ�����룺
//    if (channel1 == 15) //����ȡ������
//    {
//		moter3_move(2450);
//    }
//    else      //�������ۣ�
//    {
////		moter3_move(5400);//0918
//		moter3_move(4400);//0918
//    }
//    //4.�Ű����ƣ���Һ
//    moter2_next = aspiration_height - height;
//    moter2_move(moter2_next);
//    //5.�𵴵������λ
//    moter3_reset();
////    //6.�Ű����ƣ�������
//    moter2_next = moter2_next-200;
//    moter2_move(moter2_next);
//    //7.���ֵ������Һλ
//    moter1_next = slipAddress[channel2 - 1];
//    moter1_move(moter1_next);
//   //8.�Ű����ƣ��ƶ�8000λ�ã���Һ��
//	if(channel2==1){ //��Ӧ����Һ
//		moter3_move(4250);
//		moter2_move(8500);
//		aspiration_height=8500;
//	}
//	else
//	{  //�ƶ�9000λ�ã���Һ��
//		moter3_move(4900);
//		moter2_move(8500);
//		aspiration_height=8500;
//	}
//    //9.�𵴵�����½�����Һλ��
//	    moter3_reset();//�𵴵����λ.
//}






/**
*   @brief: �𵴲��ԣ�
*   @param:  height1: ���𵴵���½����λ�ã�height2:���𵴵���𵴸߶ȡ�
             times���𵴴�����
*   @return: void
*   @author Hang
*   @date: 2023-05-20
*/
void ShockTest(u32 height1, u32 height2, u32 times)
{	    
		aspiration_height=7000; //����Һ��ʼλ�ù��㸴λ��
		motor3Flag = 0;
		height1=height1+motor3_ResetParams;  //����𵴵������������
		Stepper_Move_S(60, 200, 0.01f, height1);
		moter3_now = height1;
		while (Stepper.status != STOP)
			while (Stepper.status != STOP)
			{
				HAL_Delay(10);
			}
			
		moter2_int(200);  //�Ű���λ	
		for (int i = 0; i < 75; i++)  //��Ϊ75�Ρ�
		{
			Stepper_Move_S(60, speedOfShock, 0.01f, -height2);
			while (Stepper.status != STOP)
			{
				HAL_Delay(10);
			}
			Stepper_Move_S(60, speedOfShock, 0.01f, height2);
			while (Stepper.status != STOP)
			{
				HAL_Delay(10);
			}
		}
			Stepper_Move_S(60, speedOfShock, 0.01f, -height2-1000);
			while (Stepper.status != STOP)
			{
				HAL_Delay(10);
			}
		moter3_now = height1-height2-1000;		
		//����ɴŰ�Ҫ��λ��
		HAL_Delay(times * 1000); //�����֮��ķ�Ӧ�ȴ�ʱ�䡣
}

//ǹͷ�����ƶ���
void gunHeadMove(u8 time, u16 step)
{
    u32 MoveStep;
    ChangeGPIOMode(2, 3); //�л���������
	step=step+motor3_ResetParams; //����ƶ�����
    if (step > moter3_now)
    {
        MOTOR_DIR(CCW);//��
        MoveStep = step - moter3_now;
    }
    else if (step < moter3_now)
    {
        MOTOR_DIR(CW);//��
        MoveStep = moter3_now - step;
    }
    else
    {
        MoveStep = 0;
    }
    MoveStep = MoveStep * 2;
    for (int i = 0; i < MoveStep; i++)
    {
        MOTOR_PUL_TOGGLE;
        ms_Delay(time);
    }
		moter3_now = step;
		ChangeGPIOMode(1, 3); //�лؼӼ���ģʽ��
}

/**
�𵴵�������ƶ���
*/



//���3�����ű����ƶ� flag�����ͣ��ƶ����롣
void moter3Move3(u8 flag, u32 params)
{
    if (flag == 0x01) //ȡǹͷ
    {
        moter3_move(4500);
		gunHeadMove(4, params);
//        moter3_move(0);
		moter3_reset();
    }
    else if (flag == 0x02) //ж�شŰ��׺��𵴵�������˶��� //
    {
		gunHeadMove(4, 4000);
//		moter3_move(params);
			moter3_reset();
    }
    else if (flag == 0x03) //��������
    {		  			
		moter3_move(4500);
		gunHeadMove(15, 5100); 
		HAL_Delay(6000); //��һ��λ�õȴ���ʱ��
		gunHeadMove(15, params);
		HAL_Delay(8000); //�ڶ���λ�õȴ���ʱ��	
		gunHeadMove(10, 4500);//
//		moter3_move(0);	
		moter3_reset();
    }
	else if(flag == 0x04) //��������2 ����ϴҺ�н��� 
	{
		moter3_move(3000);
		gunHeadMove(15, 4600); 
		HAL_Delay(8000); //��һ��λ�õȴ���ʱ��
		gunHeadMove(15, params);
		HAL_Delay(8000); //�ڶ���λ�õȴ���ʱ��	
		gunHeadMove(10, 3000);//
//		moter3_move(0);
		moter3_reset();		
	}
	else if(flag==0x05){
		gunHeadMove(10, params);
	}
}


/**********************************************************
 * @brief   ��Test��ȡ����ָ�����ݣ�����ָ�����
 * @param   
 * @author  hang
 * @date  
 **********************************************************/

void  getPraVue()
{
    uint32_t params;
    params = (R3 - 0x30) * 10000 + (R4 - 0x30) * 1000 + (R5 - 0x30) * 100 + (R6 - 0x30) * 10 + (R7 - 0x30);

    int moveStep;  //�𵴵���ƶ�����
    switch (R0)
    {
    case 0x01:                 //01����ʾ�͵���˶��й�
        switch (R1)
        {
        case 0x00://�����λ
            if (R2 == 0x01)
            {
                moter1_reset(params);
            }
            else if (R2 == 0x02)
            {
                moter2_reset(params);
            }
            else if (R2 == 0x03)
            {
                moter3_reset();
            }
            else if (R2 == 0x04) //��̨   //���õ����λ���� ������
            {
                motor1_ResetParams = params;
            }
            else if (R2 == 0x05) //�Ű�
            {
                motor2_ResetParams = params;
            }
            else if (R2 == 0x06) //��
            {
                motor3_ResetParams = params;

            }
						
						
            else if (R2 == 0x07) //��̨
            {
                motor1_ResetParams = -params;
            }
            else if (R2 == 0x08) //�Ű�
            {
                motor2_ResetParams = -params;
            }
            else if (R2 == 0x09) //��
            {
                motor3_ResetParams = -params;
            }
            break;
        case 0x01://����ƶ�          
            if (R2 == 0x01)
            {
                moter1_move(params);
            }
            else if (R2 == 0x02)
            {
                moter2_move(params);
            }
            else if (R2 == 0x03) //ʹ��S��Ӽ��٣�
            {
                if (params == 0)
                {
                    moter3_reset();
                }
                else
                {
                    moter3_move(params);
                }
            }
            else if (R2 == 0x04) //�Ƹ˵���ٶ�
            {
                motor1_Speed = params;
            }
            else if (R2 == 0x05) //�޸ĴŰ��ٶ�
            {
                motor2_Speed = params;
            }
            else if (R2 == 0x06) //�޸��𵴵���ٶ�
            {
                motor3_Speed = params;
            }
            break;
        case 0x02:

            break;
        case 0x03:

            break;
        case 0x04: //��Һ��Ԫ���ԣ�          ����������Ҫʵ�ֵĹ��ܺ�����2025��3��23��21:12:03
            if (R2 == 0x01) //��Һ�߶�
            {
                heighOfLiquid = params;
            }
            else if (R2 == 0x02) //������
            {
                moter3_position = params;
            }
            else if (R2 == 0x03) //���ô���
            {
                testTimes = params;
                LoopPickUpTest(heighOfLiquid, moter3_position, testTimes);
            }
            else if (R2 == 0x04) //������Һ��λ
            {
                channel1OfLiquid = params;
            }
            else if (R2 == 0x05) //������Һ��λ
            {
                channel2OfLiquid = params;
            }
            else if (R2 == 0x06) //������Һ��
            {
                heighOfLiquid = params;
                aspirationTest(heighOfLiquid, channel1OfLiquid, channel2OfLiquid);
            }else if(R2==0x07){
							aspirationTest_drop();
						}
            break;
        case 0x05: //����𵴲��ԣ�
            if (R2 == 0x02) //�����𵴵���ٶ�
            {
                speedOfShock = params;
            }
            else if (R2 == 0x03)//��λ��
            {
                height1OfShock = params;
            }
            else if (R2 == 0x04) //����߶�
            {
                height2OfShock = params;
            }
            else if (R2 == 0x05) //�𵴴���
            {
                timesOfShocck = params;
                ShockTest(height1OfShock, height2OfShock, timesOfShocck);
            }
            break;
						
						
        default:
            break;
        }
        break;
    case 0x02:         //��������
        switch (R1)
        {
        case 01: //��෧��
            if (R2 == 0x01) //
          {
								HAL_GPIO_WritePin(PUMP1_GPIO_Port,PUMP1_Pin,GPIO_PIN_SET);
						}
            else if (R2 == 0x02)
            {
								HAL_GPIO_WritePin(PUMP1_GPIO_Port,PUMP1_Pin,GPIO_PIN_RESET);
            }
            break;
        case 02: //�Ҳ෧��
            if (R2 == 0x01) //
            {
               HAL_GPIO_WritePin(PUMP2_GPIO_Port,PUMP2_Pin,GPIO_PIN_SET);
            }
            else if (R2 == 0x02)
            {
							HAL_GPIO_WritePin(PUMP2_GPIO_Port,PUMP2_Pin,GPIO_PIN_RESET);
            }
            break;

        default:
            break;
        }
        break;
    case 0x03:         // ��ȡһЩ��������Ϣ��
        switch (R1)
        {
        case 01:
            if (R2 == 0x01) //���ֵ��������
            {
                if (HAL_GPIO_ReadPin(IN1_GPIO_Port, IN1_Pin) == 0)
                {
                }
                else
                {


                }
            }
            else if (R2 == 0x02) //��Һ���������
            {

                if (HAL_GPIO_ReadPin(IN2_GPIO_Port, IN2_Pin) == 0)
                {


                }
                else
                {


                }
            }
            else if (R3 == 0x03) //�𵴵��������
            {
                if (HAL_GPIO_ReadPin(IN3_GPIO_Port, IN3_Pin) == 1)
                {


                }
                else
                {

                }
            }
            break;
        default:
            break;
        }
        break;
    case 0x04: //�����������
        params = (R4 - 0x30) * 1000 + (R5 - 0x30) * 100 + (R6 - 0x30) * 10 + (R7 - 0x30);
        if (R2 == 0x01) //���ó��ֵ��
        {
            if (R3 == 0x30)
            {
                motor1_ResetParams = -params;
            }
            else if (R3 == 0x31)
            {
                motor1_ResetParams = params;
            }
        }
        else if (R2 == 0x02) //���ôŰ����
        {
            if (R3 == 0x30)
            {
                motor2_ResetParams = -params;
            }
            else if (R3 == 0x31)
            {
                motor2_ResetParams = params;
            }
        }
        else if (R2 == 0x03) //�����𵴵��
        {
            if (R3 == 0x30)
            {
                motor3_ResetParams = -params;
            }
            else if (R3 == 0x31)
            {
                motor3_ResetParams = params;
            }
        }
        break;
    case 0x05: //����л�IO�ƶ�����
        gunHeadMove(R1, params);
        break;
    case 0x06:
        if (R1 == 0x01) //����
        {



        }
        else if (R1 == 0x02) //��Һ
        {



        }
        else if (R1 == 0x03) //��
        {


        }
        else    //Ĭ��״̬
        {

        }
        break;
    case 0x07:
        moter3Move3(R1, params); //�𵴵�������ƶ����ԡ�
        break;
    default:
        break;
    }
}



/**********************************************************
 * @brief   һ��������TIP  Z����Һǰ��λ-�� Z��������Һ-����λ��ͣ10S-����Һ��
 * @param   
 * @author  hang
 * @date  
 **********************************************************/
void SingleTest(u32 params, u32 motor3_position)
{
    params = 9000 - params;
    int moveStep;
    //�����´Ű��ƶ����룺
    if (params < 5000)
    {
        params = 5000;
    }
    else if (params > 9500)
    {
        params = 5000;
    }
		
		
    //1.�Ű�������ϸ�λ
    moter2_int(200);
    //2. �𵴵����λ������ϸ�λ
    motor3Flag = 1;
    Stepper_Move_S(60, 200, 0.01f, -10000);
    while (Stepper.status != STOP);
    HAL_Delay(10);
    moter3_now = 0;
    //3.�Ű��½��½�10000;
    moter2_step(10000, 0);
    //3.�Ű�����1000�������������������������Һ
    moter2_step(9000, 0);
    //4.�𵴵���½�6000�� ��Һ����̽���Լ�Һ��
    motor3Flag = 0;
    moveStep = motor3_position - moter3_now;
    Stepper_Move_S(60, 200, 0.01f, moveStep);
    moter3_now = 5300;
    HAL_Delay(2000);
    //5.�Ű���Һ
    moter2_step(params, 0);
    HAL_Delay(500);
    //6.�𵴵����λ������ϸ�λ
    motor3Flag = 1;
    Stepper_Move_S(60, 200, 0.01f, -10000);
    while (Stepper.status != STOP);
    HAL_Delay(10);
    moter3_now = 0;
    HAL_Delay(500);
//    //7.�Ű��������50����50������,��ֹҺ�����.             �ɴ��Ȳ�������
//    moter2_step(params - 50, 0);
    HAL_Delay(10000);
    //8.�Ű���������ƶ���10000��������Һ��
    moter2_step(10500, 0);
    HAL_Delay(3000);
}

/**********************************************************
 * @brief   ѭ����Һ����
 * @author  hang
 * @date  
 **********************************************************/
void LoopPickUpTest(u32 LiquidHeight, u32 Motor3_Position, u8 times)
{
    for (int i = 0; i < times; i++)
    {
        SingleTest(LiquidHeight, Motor3_Position);
    }
}

void sysInit(void){

		Queue_Init(&msgQueue);
    // �״��������գ��� currentFrame.data[0] ��ʼ��
    HAL_UART_Receive_IT(&huart1, &currentFrame.data[0], 1);
		DIR2_L;
		HAL_Delay(2000);
		EN2_H;
		EN1_H;	
		EN3_H;	
	  init();	
}




/**********************************************************
 * @brief   ��������������CAN���жϽ��յ�������
 * @author  hang
 * @date  
 **********************************************************/
void test()
{
	
		 Frame frame;
        if (Queue_Pop(&msgQueue, &frame)) {
            // ��������֡��ʾ������ӡ֡���ݣ�
            for (uint8_t i = 0; i < frame.length; i++) {
                printf("%02X ", frame.data[i]);
            }
            printf("\n");

            // �ڴ������ľ���ָ������߼�
            // ���磺if (frame.data[2] == 0x01) { ... }
						
						    R0 = frame.data[2];
								R1 = frame.data[3];
								R2 = frame.data[4];
								R3 = frame.data[5];
								R4 = frame.data[6];
								R5 = frame.data[7];
								R6 = frame.data[8];
								R7 = frame.data[9];
								getPraVue();
        }
				HAL_Delay(1000);
}


/**********************************************************
 * @brief   //��ʱ1s������
 * @author  hang
 * @date  
 **********************************************************/
void ms_Delay(uint16_t t_ms)
{
    uint32_t t = t_ms * 3127;
    while (t--);
}


/**********************************************************
 * @brief   �Բɼ���ADCֵ��ƽ��
 * @author  hang
 * @date  
 **********************************************************/
u16 Get_Adc_Average(u8 times)
{

}

/**********************************************************
 * @brief   ��ȡ�¶�ֵ��
 * @author  hang
 * @date  
 **********************************************************/
void getTemp()
{

}




/**********************************************************
 * @brief   //��ʱ����TIM_CH1 �����PWMֵ,�����Ƽ���˿�Ĺ�����
 * @author  hang
 * @date  
 **********************************************************/
void timer2()
{

	
	
	
}



/**********************************************************
 * @brief   ϵͳ��ʱ���ص������������������״̬����
 * @author  hang
 * @date  
 **********************************************************/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
		if (htim->Instance == TIM4)
    {
        TIM4Fun();
    }
    else if (htim->Instance == TIM3)
    {
        TIM3_Fun();
    }
    else if (htim->Instance == TIM1)
    {
        Speed_Decision();
    }
}

///**********************************************************
// * @brief   //�л�GPIO ���ģʽ:�����л�PWM����ͨIOģʽ��
// * @author  hang
//* @date  2025��3��17��14:46:47 
// **********************************************************/

void ChangeGPIOMode(u8 flag, u8 moterId)
{
    if (flag == 1) //�л�ΪPWMģʽ
    {
        if (moterId == 1)
        {
//               MX_TIM4_Init();
        }
        else if (moterId == 2)
        {
            MX_TIM3_Init();
        }
        else if (moterId == 3)  //�𵴵��
        {
            MX_TIM1_Init();
        }
    }
    else if (flag == 2) //�л�Ϊ��ͨIOģʽ
    {
        if (moterId == 1)
        {

        }
        else if (moterId == 2)
        {
            GPIO_InitTypeDef GPIO_InitStruct;
            __HAL_RCC_GPIOB_CLK_ENABLE();
            GPIO_InitStruct.Pin = GPIO_PIN_4;
            GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
            GPIO_InitStruct.Pull = GPIO_NOPULL;
            GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
            HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
        }
        else if (moterId == 3)
        {
            //���𵴵���л��� ��ͨIO�����С�
            GPIO_InitTypeDef GPIO_InitStruct;
            __HAL_RCC_GPIOB_CLK_ENABLE();
            GPIO_InitStruct.Pin = GPIO_PIN_8;
            GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
            GPIO_InitStruct.Pull = GPIO_NOPULL;
            GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
            HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
        }
    }
}








