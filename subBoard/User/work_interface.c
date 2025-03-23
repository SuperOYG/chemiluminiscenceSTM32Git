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
#include "PID_Realize.h"



//�ڶ���GitHub�ύ����
uint16_t github; //test
//���Ա���
uint32_t  getNumber;

//�жϼ�ʱ����
uint32_t time;


uint8_t R0, R1, R2, R3, R4, R5, R6, R7, R8, R9; //�������մ����·���ָ�

//��ǰʹ�õĴ���λ��
extern uint8_t CurUartNum;

extern uint8_t  RecieveBuffer[1];//�ݴ���յ����ַ�
extern uint8_t  Rx_end;  //ָ�������ɱ�־λ
extern uint8_t  RxLine;  //rxbuf���յ����ݳ���Ϊ:RxLine+1;
extern uint8_t  rxbuf[50];//�յ������ݴ�Ŵ�
uint8_t TextBuffer[8] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x44, 0x55, 0x55};

////CANͨѶ������
extern CAN_TxHeaderTypeDef TXHeader;
extern CAN_RxHeaderTypeDef RXHeader;
extern uint8_t CanTXmessage[8];
extern uint8_t CanRXmessage[8];
extern uint32_t pTxMailbox;

//�𵴵���ƶ�
int MoveStep;

//Can�������ݺ�����
void CAN_senddata(CAN_HandleTypeDef *hcan, uint8_t Data[8], u32 id)
{
    TXHeader.StdId = id;
    TXHeader.ExtId = 0x12345000;
    TXHeader.DLC = 8;
    TXHeader.IDE = CAN_ID_STD;
    TXHeader.RTR = CAN_RTR_DATA;
    TXHeader.TransmitGlobalTime = DISABLE;
    HAL_CAN_AddTxMessage(hcan, &TXHeader, Data, &pTxMailbox);
}

// ADCת��ֵ
__IO uint32_t ADC_ConvertedValue;
// ���ڱ���ת�������ĵ�ѹֵ
float ADC_Vol;

/**
******���ʹ�õ��м����*********
*/
//��ǰ���¶�ֵ��
float CurTemp;
//Ŀ���¶ȣ�
//float targetTemp=37;
float targetTemp = 39;

//�¶��ϴ���־λ��
u8 uploadTempFlag;

//PID����
realizePID rPID;        //pid����
realizeError rError;    //������
int TMP_PWM = 0;            //�����¶ȵ�PWM���

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
int motor1_ResetParams;
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
u16 aspiration_height=7000;
//�л����ģʽ��
void ChangeGPIOMode(u8 flag, u8 moterId);

//�ظ���Һ������
void LoopPickUpTest(u32 LiquidHeight, u32 motor3_Position, u8 times);
//�𵴲��Ժ�����
void ShockTest(u32 height1, u32 height2, u32 times);
//��ʱms:
void ms_Delay(uint16_t t_ms);


//PID��Error������ʼ������
void PidAndErrorInit()
{
    rPID.P = 2000;
    rPID.I = 120;
    rPID.D = 600;
    rPID.PWMLIMITE = 9999;    //�Ĳ���ֵ״̬��,����14.1V,���������14V;
}

//��������
void Start_TempCon(void)
{
    HAL_TIM_Base_Start_IT(&htim2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
}
//ֹͣ����
void Stop_TempCon(void)
{
    HAL_TIM_Base_Stop_IT(&htim2);
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
//    Hot_OFF;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
}



//���1 ��λ��

void moter1_int2(u16 CurPosition, u16 step)
{
    u16 moveStep = CurPosition + 400;
    DIR1_L;
    Moter1_Run(1, 1, moveStep); //10000��Ϊ�˶���Զ���Ĳ�����������
    DIR1_H;

    moter1_now = 0;
    HAL_Delay(200);
    Moter1_Run(0, 1, step); //�������أ��ز�һС����
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
void init_PA0();
/**********************************************************
 * @brief ��Ԫ��ĵ��3�Ḵλ
 * @author  hang
 * @date  
 **********************************************************/
void init()
{
    HAL_Delay(2000);
    moter2_int(200);//�Ű���λ
    if (HAL_GPIO_ReadPin(IN3_GPIO_Port, IN3_Pin) != 1) //�ж�һ���𵴵������ǰ������λ�ã�
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
//    PidAndErrorInit();
//    Start_TempCon();  //�����¿�
	
	//��ʼ���������飺
					int n = (slipAddress[1] - slipAddress[10]) / 9;
					for (int i = 2; i < 10; i++)
					{
							slipAddress[i] = slipAddress[1] - n * (i - 1);
					}	
				if(Frame_header==0x01){
				
					
				}else  if(Frame_header==0x02)
				{
					for(int i=0;i<15;i++){			
						slipAddress[i]=slipAddress[i]-115;
					}
				}else if(Frame_header==0x03){
					for(int i=0;i<15;i++){			
						slipAddress[i]=slipAddress[i]-110;
					}
				}
	
	init_PA0();//��ʼ�����ſ��ء�
	
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
    if (uploadTempFlag == 1)
    {
        getTemp();
        int a = CurTemp * 100;
        if (CurTemp < 10) //952
        {
            int a1 = a / 100;
            int a2 = a / 10 % 10;
            int a3 = a % 10;
            TextBuffer[0] = 0x02;
            TextBuffer[1] = 0x01;
            TextBuffer[2] = 0x00;
            TextBuffer[3] = 0x30;
            TextBuffer[4] = 0x30 + a1;
            TextBuffer[5] = 0x30 + a2;
            TextBuffer[6] = 0x30 + a3;
            TextBuffer[7] = 0x30;
        }
        else if (CurTemp < 100) //1952
        {
            int a1 = a / 1000;
            int a2 = a / 100 % 10;
            int a3 = a / 10 % 10;
            int a4 = a % 10;
            TextBuffer[0] = 0x02;
            TextBuffer[0] = 0x02;
            TextBuffer[1] = 0x01;
            TextBuffer[2] = 0x00;
            TextBuffer[3] = 0x30 + a1;
            TextBuffer[4] = 0x30 + a2;
            TextBuffer[5] = 0x30 + a3;
            TextBuffer[6] = 0x30 + a4;
            TextBuffer[7] = 0x30;
        }
        CAN_senddata(&hcan, TextBuffer, 0x11);
    }
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
	//1.�Ű�������Һ��ʼ��
    moter2_move(aspiration_height); //�Ƶ���Һ��ʼλ�ã��ƶ���7000������ʼʵ�顣;2023-09-11  ����Ϊ6500 �������Ƿ����¸ɾ���
    //2.���ֵ�����Ƶ�ȡҺλ��
    moter1_next = slipAddress[channel1 - 1];
    moter1_move(moter1_next);
    //3.�𵴵��������ָ�����룺
    if (channel1 == 15) //����ȡ������
    {
//		moter3_move(2600);
//		moter3_move(2390); //0918
		moter3_move(2450);
    }
    else      //�������ۣ�
    {
//		moter3_move(5400);
		moter3_move(5400);//0918
    }
    //4.�Ű����ƣ���Һ
    moter2_next = aspiration_height - height;
    moter2_move(moter2_next);
    //5.�𵴵������λ
    moter3_reset();
//    //6.�Ű����ƣ�
//    moter2_next = moter2_next;
//    moter2_move(moter2_next);
    //7.���ֵ������Һλ
    moter1_next = slipAddress[channel2 - 1];
    moter1_move(moter1_next);
   //8.�Ű����ƣ��ƶ�8000λ�ã���Һ��
	if(channel2==1){ //��Ӧ����Һ
//		moter3_move(4300);
		moter3_move(4250);
		moter2_move(9200);
		aspiration_height=9200;
		
//		moter2_move(10500);
//		aspiration_height=10500;
	}
	else
	{  //�ƶ�9000λ�ã���Һ��
		moter3_move(4900);
		moter2_move(10500);
		aspiration_height=10500;
//		moter2_move(9000);
//		aspiration_height=9000;
	}
    //9.�𵴵�����½�����Һλ��
	    moter3_reset();//�𵴵����λ.
}

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

//���յ�����CAN�ʹ��ڵ�ָ����з�������




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
            else if (R2 == 0x00)
            {

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
    case 0x02:         //��ʾ���¶ȿ����й�
        switch (R1)
        {
        case 01:
            if (R2 == 0x01) //�¿ؿ���
            {
//                PidAndErrorInit();
//                Start_TempCon();
//              PidAndErrorInit();
								HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,GPIO_PIN_SET);
            }
            else if (R2 == 0x02)
            {
							  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,GPIO_PIN_RESET);   
//                Stop_TempCon();
            }
            break;
        case 02:
            if (R2 == 0x01) //�ϴ��¶�
            {
                uploadTempFlag = 1;
            }
            else if (R2 == 0x02)
            {
                uploadTempFlag = 2;
            }
            break;
        case 03: //�趨Ŀ���¶�
            targetTemp = (R6 - 0x30) * 10 + (R7 - 0x30) + 0.5; //������0.5�㡣
            if (targetTemp > 50)
            {
                targetTemp = 25;
            }
            break;
        case 04: //�����¿�PID������s
            if (R2 == 0x01)
            {
                rPID.P = params;
                if (rPID.P > 9900)
                {
                    rPID.P = 9900;
                }
            }
            else  if (R2 == 0x02)
            {
                rPID.I = params;
                if (rPID.I > 9900)
                {
                    rPID.I = 9900;
                }
            }
            else  if (R2 == 0x03)
            {
                rPID.D = params;
                if (rPID.D > 9900)
                {
                    rPID.D = 9900;
                }
            }
            else  if (R2 == 0x04)
            {
                rPID.PWMLIMITE = params;
                if (rPID.PWMLIMITE > 9900)
                {
                    rPID.PWMLIMITE = 9900;
                }
            }
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

/**********************************************************
 * @brief   ��������������CAN���жϽ��յ�������
 * @author  hang
 * @date  
 **********************************************************/
void test()
{
    if (Rx_end == 1)
    {
        R0 = rxbuf[1];
        R1 = rxbuf[2];
        R2 = rxbuf[3];
        R3 = rxbuf[4];
        R4 = rxbuf[5];
        R5 = rxbuf[6];
        R6 = rxbuf[7];
        R7 = rxbuf[8];
        getPraVue();
        Rx_end = 0;
    }
    else if (Rx_end == 2)
    {
        if (RXHeader.StdId == Frame_header)
        {
            R0 = CanRXmessage[0];
            R1 = CanRXmessage[1];
            R2 = CanRXmessage[2];
            R3 = CanRXmessage[3];
            R4 = CanRXmessage[4];
            R5 = CanRXmessage[5];
            R6 = CanRXmessage[6];
            R7 = CanRXmessage[7];
            //1.����CAN���ߵ�ָʾGPIO�ĵ�ƽ��
            HAL_GPIO_WritePin(CAN_Signal_GPIO_Port, CAN_Signal_Pin, GPIO_PIN_SET);
            getPraVue();
            //2.����CAN���ߵ�ָʾGPIO�ĵ�ƽ��
            HAL_Delay(100);
            CAN_senddata(&hcan, TextBuffer, 0x11);
            HAL_GPIO_WritePin(CAN_Signal_GPIO_Port, CAN_Signal_Pin, GPIO_PIN_RESET);
//          HAL_Delay(100);
        }
        Rx_end = 0;
    }
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
    u32 temp_val = 0;
    u8 t;
    for (t = 0; t < times; t++)
    {
        temp_val += ADC_ConvertedValue;
//      HAL_Delay(1);
        ms_Delay(1);
    }
    return temp_val / times;
}

/**********************************************************
 * @brief   ��ȡ�¶�ֵ��
 * @author  hang
 * @date  
 **********************************************************/
void getTemp()
{
    ADC_ConvertedValue = Get_Adc_Average(6);
    ADC_Vol = (float) ADC_ConvertedValue / 4096 * 3.3; // ��ȡת����ADֵ
    float r = (ADC_Vol * 10000.0) / (3.3 - ADC_Vol);
    double result = pow(r, -0.1588);
    CurTemp = 609.4 * result - 116.1;
}




/**********************************************************
 * @brief   //��ʱ����TIM_CH1 �����PWMֵ,�����Ƽ���˿�Ĺ�����
 * @author  hang
 * @date  
 **********************************************************/
void timer2()
{
    uint16_t RealTemp;
    getTemp();
	if(Frame_header==0x01||Frame_header==0x02){
		targetTemp=37;
	}
    rError.Current_Error = targetTemp - CurTemp;  //��õ�ǰ���
    TMP_PWM = (int)PID_Realize(&rError, &rPID);   //����PID������������PWM
    if (TMP_PWM < 0)
    {
        TMP_PWM = 0;
    }
//    TMP_PWM = rPID.PWMLIMITE - TMP_PWM; //ռ�ձ�ԽС����ЧӦ�ܵ�ͨ����Խ�󣬴˴�������    ���������������ˣ��ھ��Ӽ���ϲ���Ҫ����
    __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, TMP_PWM);
}



/**********************************************************
 * @brief   ϵͳ��ʱ���ص������������������״̬����
 * @author  hang
 * @date  
 **********************************************************/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    /* USER CODE BEGIN Callback 0 */
    static uint32_t num = 0;
    static uint32_t num1 = 0;
    /* USER CODE END Callback 0 */
    if (htim->Instance == TIM2) // ��ʱ��2����ַ
    {
        // �Զ���Ӧ�ó���
        time++;
        num++;
////        //��ʱ����TIM2 ��ʱ��������1s����һ�ζ�ʱ���жϡ�
        if (num == 100)
        {
            timer2();
            num = 0;
        }
    }
    else if (htim->Instance == TIM4)
    {
        // �Զ���Ӧ�ó���
        num1++;
        if (num1 == 1000) // ÿ1��LED�Ʒ�תһ��
        {
            num1 = 0;
        }
        TIM4Fun();
    }
    else if (htim->Instance == TIM3)
    {
        TIM3_Fun();
    }
    else if (htim->Instance == TIM1)
    {
//      TIM1_Fun();
        Speed_Decision();
    }
    /* USER CODE END Callback 1 */
}

/**********************************************************
 * @brief   //�л�GPIO ���ģʽ:�����л�PWM����ͨIOģʽ��
 * @author  hang
* @date  2025��3��17��14:46:47 
 **********************************************************/

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


/**********************************************************
 * @brief   ���¿ص�PWM���������������Ϊ ��ͨIO ���ģʽ������MOSͨ��
 * @param
 * @author  hang
 * @date  
 **********************************************************/
void init_PA0(){
		      GPIO_InitTypeDef GPIO_InitStruct;
            __HAL_RCC_GPIOB_CLK_ENABLE();
            GPIO_InitStruct.Pin = GPIO_PIN_0;
            GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
            GPIO_InitStruct.Pull = GPIO_NOPULL;
            GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
            HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}






