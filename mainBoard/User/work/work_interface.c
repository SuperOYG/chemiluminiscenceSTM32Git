#include "work_interface.h"
#include "eeprom.h"

#include "can.h"
#include "i2c.h"
//#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "motor.h"

#include "S_Moter1.h"  


//���Ա���
uint32_t  getNumber;

//�жϼ�ʱ����
uint32_t time;

uint8_t R0, R1, R2, R3, R4, R5, R6, R7, R8, R9; //�������մ����·���ָ�
//����ͨѶ����
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;

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



//Can�������ݺ�����
void CAN_senddata(CAN_HandleTypeDef *hcan, uint8_t Data[8])
{
    TXHeader.StdId = 0x00000108;
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


/*
*  *********************************���ʹ�õ��м����*******************************
*/
u32 moter3_now;      //Z������ǰλ��
u32 moter3_next;     //Z������ǰλ��

u8 motor3Flag; //  Z������λ��־λ

u32 moter1_now;      //X������ǰλ��
u32 moter1_next;     //X������ǰλ��


//��ǰ�������Լ��Ŀ�����
uint8_t NumberOfCards;
//��ǰ����״̬
uint8_t testStatus;
//��ǰ���Ե��Լ��Ŀ������
uint8_t testIndex;

//ת���ƶ��ĳ�ʼ����
uint8_t StartStep;

//ת���ƶ��ļ������
uint8_t SingleStep;

//����״̬��־λ  1���п� 0���޿�
uint8_t card1_flag;
uint8_t card2_flag;
uint8_t card3_flag;
uint8_t card4_flag;
uint8_t card5_flag;
uint8_t card6_flag;


uint16_t Ka_now;  //��⾵ͷ��ǰλ��
uint16_t Ka_next;//��⾵ͷ��һ��λ��

u32 Scan_V[380];     //����ԭʼ����
u32 Scan_VH[380];    //ԭʼ���ݸ�λ
u32 Scan_VL[380];    //ԭʼ���ݵ�λ
float Scan_Float_V[380];

float ADC_ConvertedValueLocal;

float Result_1;                      //��Ǽ����
float Result_2;                      //��Ǽ����
float Result_3;                      //��Ǽ����
u8 state = 0;  //�ж��Ƿ��⵽��������Լ�������
u8 ResultCount;                 //�����������: ������ ResultCount=1; ������ ResultCount=2; ������ ResultCount=3;
extern u8 UART_Code[7]; //����ֵ
//////////////////////////////////////////////
//����֡ͷ
/////////////////////////
void USART_SendByte(uint8_t data){
	if(CurUartNum==1){
		Usart1_SendByte(data);
	}else if(CurUartNum==2){
		Usart2_SendByte(data);
	}
	
}

void send_front()
{
	
	USART_SendByte(0xAA);
	USART_SendByte(0x55);
//	Usart1_SendByte(0xAA);
//	Usart1_SendByte(0x55);
	
	
//    Usart_SendByte(&huart1, 0xAA);
//    Usart_SendByte(&huart1, 0x55);
}
//////////////////////////////////////////////
//����֡β
/////////////////////////
void send_end()
{
	USART_SendByte(0xFF);
	USART_SendByte(0xFE);
	
//    Usart1_SendByte(0xFF);
//    Usart1_SendByte(0xFE);
}



void Open_Scanning(void)//��������
{
    Usart_SendByte(&huart3, 0x02);
    Usart_SendByte(&huart3, 0x01);
    Usart_SendByte(&huart3, 0x01);
    Usart_SendByte(&huart3, 0x01);
    Usart_SendByte(&huart3, 0x00);
    Usart_SendByte(&huart3, 0x00);
    Usart_SendByte(&huart3, 0x00);
    Usart_SendByte(&huart3, 0x00);
    Usart_SendByte(&huart3, 0x00);
    Usart_SendByte(&huart3, 0x00);
    Usart_SendByte(&huart3, 0x00);
    Usart_SendByte(&huart3, 0x00);
    Usart_SendByte(&huart3, 0x00);
    Usart_SendByte(&huart3, 0x00);
    Usart_SendByte(&huart3, 0x03);
    Usart_SendByte(&huart3, 0xF8);
}

void Close_Scanning(void)//�ر�������
{
    Usart_SendByte(&huart3, 0x02);
    Usart_SendByte(&huart3, 0x01);
    Usart_SendByte(&huart3, 0x01);
    Usart_SendByte(&huart3, 0x00);
    Usart_SendByte(&huart3, 0x00);
    Usart_SendByte(&huart3, 0x00);
    Usart_SendByte(&huart3, 0x00);
    Usart_SendByte(&huart3, 0x00);
    Usart_SendByte(&huart3, 0x00);
    Usart_SendByte(&huart3, 0x00);
    Usart_SendByte(&huart3, 0x00);
    Usart_SendByte(&huart3, 0x00);
    Usart_SendByte(&huart3, 0x00);
    Usart_SendByte(&huart3, 0x00);
    Usart_SendByte(&huart3, 0x03);
    Usart_SendByte(&huart3, 0xF9);
}


/**

    ��⸴λ״̬
*/
void rd_io()
{
    u8 step1 = 30;
    u8 step2 = 30;
//  HAL_GPIO_ReadPin(IN0_GPIO_Port,IN0_Pin)
    if (HAL_GPIO_ReadPin(IN1_GPIO_Port, IN1_Pin) == KEY_OFF) //�жϲ��Թ�������Ƿ�λ
    {
        EN1_H;
        DIR1_L;
//      GENERAL_TIM_Init();                         /*��ʱ����ʼ��*/
//      TIM_Cmd(GENERAL_TIM, DISABLE);              //�رն�ʱ��
//      TIM_Cmd(GENERAL_TIM, ENABLE);               //������ʱ��
        time = 0;
        while (step1)
        {
            if (time == 80)
            {
                time = 0;
                CP1_TOGGLE;
                step1--;
            }
        }
        EN1_L;
    }
    if (HAL_GPIO_ReadPin(IN1_GPIO_Port, IN1_Pin) == KEY_OFF) //�жϻ�����������Ƿ�λ
    {
        EN2_H;
        DIR2_L;
//      GENERAL_TIM_Init();                         /*��ʱ����ʼ��*/
//      TIM_Cmd(GENERAL_TIM, DISABLE);              //�رն�ʱ��
//      TIM_Cmd(GENERAL_TIM, ENABLE);               //������ʱ��
        time = 0;
        while (step2)
        {
            if (time == 80)
            {
                time = 0;
                CP2_TOGGLE;
                step2--;
            }
        }
        EN2_L;
    }
}

void Scan_card1()  //------6---------
{
    if (HAL_GPIO_ReadPin(IN2_GPIO_Port, IN2_Pin) == ON)
    {
        card6_flag = 1;
		printf("����1�п�");
    }
    else{
	
        card6_flag = 0;
		printf("����1�޿�");
	}
}

void Scan_card()
{
    Scan_card1();
    //while(card1_flag=1|card2_flag=1|card3_flag=1|card4_flag=1|card5_flag=1|card6_flag=1)
	HAL_Delay(20);
	send_front();
//	USART_SendByte(0xEE);
	USART_SendByte(0x04);
   USART_SendByte(card1_flag);
   USART_SendByte(card2_flag);
   USART_SendByte(card3_flag);
   USART_SendByte(card4_flag);
   USART_SendByte(card5_flag);	
   USART_SendByte(card6_flag);
//   	USART_SendByte(0xFF);
	send_end();
}

//������ʼ��
void init()
{
//	HAL_Delay(35000);
//    Scan_card();
	printf("system init");
    Motor_init();//�����λ
    Read_equipment_parament(); //��ȡ�������
	printf("system init");
}

void Read_AD_All(unsigned int mmm)
{
    float V[100];      //��100��,ȡ��ֵ;
    float max = 0, min = 0;
    /*  ���ӵ��βɵ������ȡƽ��ֵ  */
    for (u8 i = 0; i < 100; i++)
    {
        V[i] = (float) ADC_ConvertedValue / 4096 * 3.3;
    }
    for (u8 a = 0; a < 100; a++)
    {
        if (max > V[a])
        {
            max = V[a];   //ȡ��һ�����ֵ
        }
    }
    for (u8 b = 0; b < 100; b++)
    {
        if (min < V[b])
        {
            min = V[b];   //ȡ��һ�����ֵ
        }
    }

    ADC_ConvertedValueLocal = 0;
    for (u8 i = 0; i < 100; i++)
    {
        ADC_ConvertedValueLocal = ADC_ConvertedValueLocal + V[i];
    }
    ADC_ConvertedValueLocal = ADC_ConvertedValueLocal - max - min;  //�������ֵ�����ֵ
    Scan_V[mmm] = (ADC_ConvertedValueLocal / 98) * 10000;
    Scan_Float_V[mmm] = ADC_ConvertedValueLocal / 98;
}




void Scan_win()   //ɨ�������Լ�����ɫ��   flag == 1: PC���,����762�����ݸ�PC��;
//                      flag == 2: ��Ļ���;
{
    u8 fit_num = 10;     //�趨��ϴ���
//    float max = 0, min = 0;
	EN1_H;
    for (int i = 0; i < 380; i++)  //����ÿ���� 4 �� , ���� 1 �μ��;
    {
        //Բ����� ÿ���� 6��
//        KaMoter_Step(Moter_speed + 20, X_step + 4 * i);
//        Ka1Moter_RunStep(1, equipment_parament_6 - 23* i);  //�ñ��滻2023-0331
        Read_AD_All(i);
    }
//	EN1_H;
    /***************** �˴����������˲�����     5 �� ȡ��ֵ ǰ���ȡ5��    *****************/
    for (u8 j = 0; j < fit_num; j++) //����10�����,����ƽ���ȽϺ�;
    {
        for (u32 i = 0; i < 380; i++)
        {
            if (i < 2)  // 0,1
            {
                Scan_V[i] = (Scan_V[i]   + Scan_V[i + 1] + Scan_V[i + 2] + Scan_V[i + 3] + Scan_V[i + 4]) / 5;
            }
            else if ((i > 1) && (i < 378)) //  2 ---- 377
            {
                Scan_V[i] = (Scan_V[i - 2] + Scan_V[i - 1] + Scan_V[i]   + Scan_V[i + 1] + Scan_V[i + 2]) / 5;
            }
            else if (i == 378)
            {
                Scan_V[i] = (Scan_V[i - 3] + Scan_V[i - 2] + Scan_V[i - 1] + Scan_V[i]   + Scan_V[i + 1]) / 5;
            }
            else if (i == 379)
            {
                Scan_V[i] = (Scan_V[i - 4] + Scan_V[i - 3] + Scan_V[i - 2] + Scan_V[i - 1] + Scan_V[i]) / 5;
            }
            else ;
        }
    }

    for (u8 j = 0; j < fit_num; j++) //����10�����,����ƽ���ȽϺ�;
    {
        for (u32 i = 0; i < 380; i++)
        {
            if (i < 2)  // 0,1
            {
                Scan_Float_V[i] = (Scan_Float_V[i]  +  Scan_Float_V[i + 1] + Scan_Float_V[i + 2] + Scan_Float_V[i + 3] + Scan_Float_V[i + 4]) / 5;
            }
            else if ((i > 1) && (i < 378)) //  2 ---- 377
            {
                Scan_Float_V[i] = (Scan_Float_V[i - 2] + Scan_Float_V[i - 1] + Scan_Float_V[i]   + Scan_Float_V[i + 1] + Scan_Float_V[i + 2]) / 5;
            }
            else if (i == 378)
            {
                Scan_Float_V[i] = (Scan_Float_V[i - 3] + Scan_Float_V[i - 2] + Scan_Float_V[i - 1] + Scan_Float_V[i]   + Scan_Float_V[i + 1]) / 5;
            }
            else if (i == 379)
            {
                Scan_Float_V[i] = (Scan_Float_V[i - 4] + Scan_Float_V[i - 3] + Scan_Float_V[i - 2] + Scan_Float_V[i - 1] + Scan_Float_V[i]) / 5;
            }
            else ;
        }
    }
}


//void Test_Result(u8 num)    //num:����Ŀ�м��������;
//{
//  u32 Area;                                  //ȡֵ��Χ
//  u32 t[8]={0};                              //��Ǹ�����ֵ���յ�λ��
//  u32 MaxX[4]={0};                          //��Ǹ�����ֵ��X����
//  float MaxV[4]={0};                          //��Ǹ�����ֵ����
//  float MaxArea[4]={0};                       //��Ǹ�����ֵ�������
//  float MinArea[4]={0};                       //��Ǹ�����ֵ����ı�������
//  float a,b,c,d;
//
//  switch(num)
//    {
//      case 1:   //һ�������,������ֵ(MaxV1,MaxV2);
//        t[0] = equipment_parament_3;                  //��һ����ֵ�����
//        t[1] = equipment_parament_4;                  //��һ����ֵ���յ�
//        t[2] = equipment_parament_4+1;                //�ڶ�����ֵ�����
//        t[3] = equipment_parament_5;                  //�ڶ�����ֵ���յ�
//        Area = equipment_parament_21;
//
//        MaxV[0] = Scan_Float_V[t[0]];
//        for(u16 i=t[0];i<t[1];i++)
//          {
//            if(Scan_Float_V[i] > MaxV[0])
//              {MaxV[0] = Scan_Float_V[i]; MaxX[0] = i;}  //��һ����ֵ �Լ� ��һ����ֵ��X����
//          }
//        MaxV[1] = Scan_Float_V[t[2]];
//        for(u16 j=t[2];j<t[3];j++)
//          {
//            if(Scan_Float_V[j] > MaxV[1])
//              {MaxV[1] = Scan_Float_V[j]; MaxX[1] = j;}  //�ڶ�����ֵ �Լ� �ڶ�����ֵ��X����
//          }
//
//        if(Area == 0)
//          {
//            MaxArea[0] = MaxV[0];MinArea[0] = 0;
//            MaxArea[1] = MaxV[1];MinArea[1] = 0;
//          }
//        else
//          {
//            MaxArea[0] = 0;
//            for(u16 mmm = MaxX[0] - Area; mmm < MaxX[0] + Area - 1;mmm++)
//              {MaxArea[0] = MaxArea[0] + Scan_Float_V[mmm];}
//            MinArea[0] = Scan_Float_V[MaxX[0] - Area] * Area * 2;
//
//            MaxArea[1] = 0;
//            for(u16 mmm = MaxX[1] - Area;mmm < MaxX[1] + Area - 1;mmm++)
//              {MaxArea[1] = MaxArea[1] + Scan_Float_V[mmm];}
//            MinArea[1] = Scan_Float_V[MaxX[1] - Area] * Area * 2;
//          }
//
//        a =  MaxArea[0];// - MinArea[0];
//        b =  MaxArea[1];// - MinArea[1];

//        Result_1 = (float)a/b;  //T1/C
//
////        Show_ItemName(0x01,0x12);           //��ʾ ����Ŀ��
////                HAL_Delay(50);
////        Show_Result(0x01,0x13,Result_1);    //��ʾ���1     T1/C
//        HAL_Delay(50);
//        //Show_unit(0x01,0x14);      //��ʾ��λ
//      break;
//
//      case 2:               //���������,������ֵ(MaxV1,MaxV2,MaxV3);
//        t[0] = CurvePeak_Start1;                  //��һ����ֵ�����0
//        t[1] = CurvePeak_End1;                  //��һ����ֵ���յ�100
//        t[2] = CurvePeak_Start2;                //�ڶ�����ֵ�����100
//        t[3] = CurvePeak_End2;                  //�ڶ�����ֵ���յ�200
//        t[4] = CurvePeak_Start3;                //��������ֵ�����200
//        t[5] = CurvePeak_End3;                  //��������ֵ���յ�255
//        Area = referenceValue;
//
//        MaxV[0] = Scan_Float_V[t[0]];
//        for(u16 i=t[0];i<t[1];i++)
//          {
//            if(Scan_Float_V[i]-MaxV[0])
//              {MaxV[0] = Scan_Float_V[i]; MaxX[0] = i;}  //��һ����ֵ �Լ� ��һ����ֵ��X����
//          }
//        MaxV[1] = Scan_Float_V[t[2]];
//        for(u16 j=t[2];j<t[3];j++)
//          {
//            if(Scan_Float_V[j] > MaxV[1])
//              {MaxV[1] = Scan_Float_V[j]; MaxX[1] = j;}  //�ڶ�����ֵ �Լ� �ڶ�����ֵ��X����
//          }
//        MaxV[2] = Scan_Float_V[t[4]];
//        for(u16 k=t[4];k<t[5];k++)
//          {
//            if(Scan_Float_V[k] > MaxV[2])
//              {MaxV[2] = Scan_Float_V[k]; MaxX[2] = k;}  //��������ֵ �Լ� ��������ֵ��X����
//          }
//
//        if(Area == 0)
//          {
//            MaxArea[0] = MaxV[0];MinArea[0] = 0;
//            MaxArea[1] = MaxV[1];MinArea[1] = 0;
//            MaxArea[2] = MaxV[2];MinArea[2] = 0;
//          }
//        else
//          {
//            MaxArea[0] = 0;
//            for(u16 mmm=MaxX[0]-Area;mmm<MaxX[0]+Area-1;mmm++){MaxArea[0] = MaxArea[0] + Scan_V[mmm];}
//            MinArea[0] = Scan_Float_V[MaxX[0]-Area] * 60;
//
//            MaxArea[1] = 0;
//            for(u16 mmm=MaxX[1]-Area;mmm<MaxX[1]+Area-1;mmm++){MaxArea[1] = MaxArea[1] + Scan_V[mmm];}
//            MinArea[1] = Scan_Float_V[MaxX[1]-Area] * 60;
//
//            MaxArea[2] = 0;
//            for(u16 mmm=MaxX[2]-Area;mmm<MaxX[2]+Area-1;mmm++){MaxArea[2] = MaxArea[2] + Scan_V[mmm];}
//            MinArea[2] = Scan_Float_V[MaxX[2]-Area] * 60;
//          }
//
//        a = MaxArea[0];// - MinArea[0];
//        b = MaxArea[1];// - MinArea[1];
//        c = MaxArea[2];// - MinArea[2];
//
//        Result_1 = (float)a/c;  //T1/C
//        Result_2 = (float)b/c;  //T2/C
//
////        Show_ItemName(0x01,0x12);           //��ʾ ����Ŀ��
////                HAL_Delay(50);
////        Show_Result(0x01,0x13,Result_1);    //��ʾ���1     T1/C
//              HAL_Delay(50);
//        //Show_unit(0x01,0x14);               //��ʾ��λ
//
////        Show_ItemName(0x01,0x15);           //��ʾ ����Ŀ��
////                HAL_Delay(50);
////        Show_Result(0x01,0x16,Result_2);    //��ʾ���1     T2/C
//              HAL_Delay(50);
//        //Show_unit(0x01,0x17);               //��ʾ��λ
//      break;

//      case 3:                       //���������,������ֵ(MaxV1,MaxV2,MaxV3,MaxV4);
//        t[0] = equipment_parament_10;                  //��һ����ֵ�����
//        t[1] = equipment_parament_11;                  //��һ����ֵ���յ�
//        t[2] = equipment_parament_11+1;                //�ڶ�����ֵ�����
//        t[3] = equipment_parament_12;                  //�ڶ�����ֵ���յ�
//        t[4] = equipment_parament_12+1;                //��������ֵ�����
//        t[5] = equipment_parament_13;                  //��������ֵ���յ�
//        t[6] = equipment_parament_13+1;                //���ĸ���ֵ�����
//        t[7] = equipment_parament_14;                  //���ĸ���ֵ���յ�
//        Area = equipment_parament_23;
//
//        MaxV[0] = Scan_Float_V[t[0]];
//        for(u16 i=t[0];i<t[1];i++)
//          {
//            if(Scan_Float_V[i] > MaxV[0]){MaxV[0] = Scan_Float_V[i]; MaxX[0] = i;}  //��һ����ֵ �Լ� ��һ����ֵ��X����
//          }
//        MaxV[1] = Scan_Float_V[t[2]];
//        for(u16 j=t[2];j<t[3];j++)
//          {
//            if(Scan_Float_V[j] > MaxV[1]){MaxV[1] = Scan_Float_V[j]; MaxX[1] = j;}  //�ڶ�����ֵ �Լ� �ڶ�����ֵ��X����
//          }
//        MaxV[2] = Scan_Float_V[t[4]];
//        for(u16 k=t[4];k<t[5];k++)
//          {
//            if(Scan_Float_V[k] > MaxV[2]){MaxV[2] = Scan_Float_V[k]; MaxX[2] = k;}  //��������ֵ �Լ� ��������ֵ��X����
//          }
//        MaxV[3] = Scan_Float_V[t[6]];
//        for(u16 l=t[6];l<t[7];l++)
//          {
//            if(Scan_Float_V[l] > MaxV[3]){MaxV[3] = Scan_Float_V[l]; MaxX[3] = l;}  //��������ֵ �Լ� ��������ֵ��X����
//          }
//
//        if(Area == 0)
//          {
//            MaxArea[0] = MaxV[0];MinArea[0] = 0;
//            MaxArea[1] = MaxV[1];MinArea[1] = 0;
//            MaxArea[2] = MaxV[2];MinArea[2] = 0;
//            MaxArea[3] = MaxV[3];MinArea[3] = 0;
//          }
//        else
//          {
//            MaxArea[0] = 0;
//            for(u16 mmm=MaxX[0]-Area;mmm<MaxX[0]+Area-1;mmm++){MaxArea[0] = MaxArea[0] + Scan_V[mmm];}
//            MinArea[0] = Scan_Float_V[MaxX[0]-Area] * 60;
//
//            MaxArea[1] = 0;
//            for(u16 mmm=MaxX[1]-Area;mmm<MaxX[1]+Area-1;mmm++){MaxArea[1] = MaxArea[1] + Scan_V[mmm];}
//            MinArea[1] = Scan_Float_V[MaxX[1]-Area] * 60;
//
//            MaxArea[2] = 0;
//            for(u16 mmm=MaxX[2]-Area;mmm<MaxX[2]+Area-1;mmm++){MaxArea[2] = MaxArea[2] + Scan_V[mmm];}
//            MinArea[2] = Scan_Float_V[MaxX[2]-Area] * 60;
//
//            MaxArea[3] = 0;
//            for(u16 mmm=MaxX[3]-Area;mmm<MaxX[3]+Area-1;mmm++){MaxArea[3] = MaxArea[3] + Scan_V[mmm];}
//            MinArea[3] = Scan_Float_V[MaxX[3]-Area] * 60;
//          }
//
//        a = MaxArea[0];// - MinArea[0];
//        b = MaxArea[1];// - MinArea[1];
//        c = MaxArea[2];// - MinArea[2];
//        d = MaxArea[3];// - MinArea[3];
//
//        Result_1 = (float)a/d;  //T1/C
//        Result_2 = (float)b/d;  //T2/C
//        Result_3 = (float)c/d;  //T2/C
//
////        Show_ItemName(0x01,0x12);           //��ʾ ����Ŀ��
////                HAL_Delay(50);
////        Show_Result(0x01,0x13,Result_1);    //��ʾ���1     T1/C
//              HAL_Delay(50);
//        //Show_unit(0x01,0x14);               //��ʾ��λ
//
//        Show_ItemName(0x01,0x15);           //��ʾ ����Ŀ��
//              HAL_Delay(50);
//        Show_Result(0x01,0x16,Result_2);    //��ʾ���1     T2/C
//              HAL_Delay(50);
//        //Show_unit(0x01,0x17);               //��ʾ��λ
//
//        Show_ItemName(0x01,0x18);           //��ʾ ����Ŀ��
//              HAL_Delay(50);
//        Show_Result(0x01,0x19,Result_3);    //��ʾ���2     T3/C
//              HAL_Delay(50);
//        //Show_unit(0x01,0x1A);               //��ʾ��λ
//      break;
//      default:     break;
//    }
//      Save_Data(num);  //num ��ǹ��м��������
//}

void Test_TC(u8 n,u8 flag)    //ɨ���Լ��� �Ӵ�
{
//    u8 SampleNum_H, SampleNum_L;
    HAL_Delay(500);    //0.5s
//    KaMoter_Step(Moter_speed, equipment_parament_2);  //���е� ��ֽ����ⴰ ���
//    Moter_MoveToChannel(n, 1);//
    LED_H;   //����
    HAL_Delay(800);   //�ȴ���Դ�����ȶ�  ʱ�䲻С��700ms
    Scan_win();   //ɨ�������Լ��Ӵ�,ȡ�� 380 ���������;
    HAL_Delay(500);     //0.05s
    LED_L;  //�ص�
//    Moter_DropCard();//
    HAL_Delay(500);    //0.5s
//    Ka_InitBack(Moter_speed); //�Լ������λ ,ж���Լ���;


//    Ka_InitFont(Moter_speed); //�Լ�����ǰ��λ;
    //item = 1;//���������ԣ��������ʱ����ֵȥ��

	Code_flag=flag;
    if (Code_flag == 0x01)      //�������رյ������Ĭ��������ֵ;
    {
        state = 1; //��⵽�Լ���
        ResultCount = 1;
//        Test_Result(ResultCount);  ����Ǽ����ֵ�Ƚ�ֵ����������Ƿ���android��Ļʵ�֡�
        for (int i = 0; i < 380; i++) ////��7������С����,�˴������������С100����ʾ;
        {
            //Scan_VH[i] = Scan_V[i]/100;
            Scan_VH[i] = Scan_V[i] >> 8;
            Scan_VL[i] = Scan_V[i] & 0x00ff;
        }

        //���ͻ�����������ָ�����ַ���     EE B1 32 00 07 00 01 00 02 00
        send_front();
//        Usart_SendByte(&huart1, 0x07);
		USART_SendByte(0x07);
        for (u32 i = 0; i < 380; i++) //��7������С����,�˴��ɲ����ͼ�����ݵ�λ;
        {
//            Usart_SendByte(&huart1, Scan_VH[i]);
//            Usart_SendByte(&huart1, Scan_VL[i]);
//			Usart1_SendByte(Scan_VH[i]);
//			Usart1_SendByte(Scan_VL[i]);
			USART_SendByte(Scan_VH[i]);
			USART_SendByte(Scan_VL[i]);
//			
//			Usart2_SendByte(Scan_VH[i]);
//			Usart2_SendByte(Scan_VL[i]);
			
        }
        send_end();
    }
//  else if (Code_flag == 0x00)  //�������򿪵����
//    {
//        switch (item)
//        {
//        case CRP:   //1   �ݶ�����Ŀ��2���ߣ�1�������
//            state = 1; //��⵽�Լ���
//            ResultCount = 1;
//            Test_Result(ResultCount);
//            for (int i = 0; i < 380; i++) ////��7������С����,�˴������������С100����ʾ;
//            {
//                //Scan_VH[i] = Scan_V[i]/100;
//                Scan_VH[i] = Scan_V[i] >> 8;
//                Scan_VL[i] = Scan_V[i] & 0x00ff;
//            }
//            //���ͻ�����������ָ�����ַ���     EE B1 32 00 07 00 01 00 02 00
//            send_front();
//            Usart_SendByte(&huart3, 0x07);
//            for (u32 i = 0; i < 380; i++) //��7������С����,�˴��ɲ����ͼ�����ݵ�λ;
//            {
//                Usart_SendByte(&huart3, Scan_VH[i]);
//                Usart_SendByte(&huart3, Scan_VL[i]);
//            }
//            send_end();
//            break;
//        case PG1:   //2  �ݶ�����Ŀ��3���ߣ�2�������
//            state = 1; //��⵽�Լ���
//            ResultCount = 2;
//            Test_Result(ResultCount);
//            break;
//        case PG2:   //3  �ݶ�����Ŀ��4���ߣ�3�������
//            state = 1; //��⵽�Լ���
//            ResultCount = 3;
//            Test_Result(ResultCount);
//            break;
//        default:
//            item = 0;
//            state = 0; //δ��⵽�Լ���
//            break;
//        }
//    }
}



//void Immediately_Test(u8 flag)
//{
//    if (flag == 0x00)
//    {
//        for (u16 a = 0; a < IncubationTime; a++)
//        {
//            HAL_Delay(1000);   //��ʱ����ʱ��
//        }
//    }
//    Code_flag = I2C_get2Byte(Code_flag_address);  //��ȡ���뿪��״̬
//    if (Code_flag == 0x00)  //���뿪
//    {
//        //�˴��ж������Ƿ�ɨ��ɹ������ж��Ƿ���ID����Ϣ��
//        Code_Parameter[0] = 0x00;
//        //item = 0;                        //��ʼ����������ϢΪ��
//        Open_Scanning();                                   //
//        HAL_Delay(300);
//        KaMoter_Step(Moter_speed, equipment_parament_1);  //���е�ɨ������λ��
//        HAL_Delay(100);
//        Close_Scanning();
//        Ka_InitFont(Moter_speed);                 //ǰ��Ѱ��
//        //Save_CodeData(item);

//        if (item == 0x01 | item == 0x02 | item == 0x03)  //ɨ�赽������Ϣ
//        {
//            Set_Form_Parameter();                  //����ַ�����ݱ��浽�β��У���ʱʹ��
//            show_screen_id01_ephemeraldata();      //��ʾ��ʱ����(��ˮ�ŵ�)
//            HAL_Delay(50);
//            Test_TC();
//            //HAL_Delay(100);
//            Address_Zero();//���ø�����Ϣ�洢��ַ���׵�ַ Ϊ0
//            //Delete_SampleNum();
//            Moter_EnNo();
//        }
//        else if (item == 0x00)   //δɨ�赽������Ϣ
//        {
//            //Delete_screen_id01();    //������Խ���
//            Show_Waring01();
//            Set_Form_Parameter();                  //����ַ�����ݱ��浽�β��У���ʱʹ��
//            //show_screen_id01_ephemeraldata();      //��ʾ��ʱ����(��ˮ�ŵ�)
//            HAL_Delay(100);
//            if (flag_tt == 1)
//            {
//                item = 1;
//                Test_TC();
//                item = 0;
//                //HAL_Delay(100);
//                Address_Zero();//���ø�����Ϣ�洢��ַ���׵�ַ Ϊ0
//                //Delete_SampleNum();
//                Moter_EnNo();
//            }
//            else
//            {
//                Moter_EnOk();
//                Ka_InitFont(Moter_speed);
//                HAL_Delay(300);
//                Moter_EnNo();
//                LED_L;
//            }
//        }
//    }
//    else if (Code_flag == 0x01) //�����
//    {
//        Moter_EnOk();
//        HAL_Delay(300);
//        //Save_DetectionTime();

//        KaMoter_Step(Moter_speed, 2000);  //��ǰ����һ�ξ���
//        HAL_Delay(100);
//        Ka_InitFont(Moter_speed);                 //ǰ��Ѱ��,��λ;

//        Set_Form_Parameter();                  //����ַ�����ݱ��浽�β��У���ʱʹ��
//        show_screen_id01_ephemeraldata();      //��ʾ��ʱ����(��ˮ�ŵ�)
//        Test_TC();
//        Address_Zero();//���ø�����Ϣ�洢��ַ���׵�ַ Ϊ0
//        Moter_EnNo();
//    }
//}



void PC_test(void)    //PC�����
{
    float PC_data[380];

    Motor_init();//�����ʼ��
//    Moter_EnOk();     //���ʹ��
    HAL_Delay(500);   //5s �ȴ���Դ�����ȶ�
//    moter_Zero();                                                           //��ǰ��λ���
//    KaMoter_Step(Moter_speed, equipment_parament_2);              //���е� ��ֽ����ⴰ ���
    Ka2Moter_Step(MotorY_speed, equipment_parament_6);          //���е� ��ֽ����ⴰ ���
//  Motor
    u8 Channel = R4; //��ȡ����ͨ��λ
//    Moter_MoveToChannel(Channel, 0);
    LED_H;    //����
    HAL_Delay(800);   //�ȴ���Դ�����ȶ�  ʱ�䲻С��700ms
    Scan_win();   //ɨ�������Լ��Ӵ�,ȡ�� 380 ���������;
    HAL_Delay(500);     //0.5s
    LED_L;  //�ص�
    for (u32 i = 0; i < 380; i++)
    {
        PC_data[i] = (long double)Scan_V[i] * 0.0001;
    }

    if (R2 == 0x01) //�ж��Ƿ�ж���Լ���   01:ж��;   00:��ж��;
    {
        HAL_Delay(100);    //0.1s
//        Ka_InitBack(Moter_speed);
//        Moter_DropCard();  //ж��
    }
    HAL_Delay(100);        //0.1s
//    Ka_InitFont(Moter_speed);
    /****************��ӡ�����м����ѹֵ****************/
    if (R3 == 0x01) //�ж����ݷ�������   =0x01 :��������
    {
        //for(int i=0;i<380;i++)
        for (int i = 3; i < 377; i++)
        {
            HAL_Delay(10);   //0.01s
            //printf("%d \r\n",Scan_V[i]);
            printf("%f \r\n", PC_data[i]);
        }
    }
    /****************��ӡ�����м����ѹֵ****************/
    else if (R3 == 0x00)  //�ж����ݷ�������   =0x00 :���͸ߡ���λ��760 + 2 ������
    {
        for (int i = 0; i < 380; i++) //һ�����ݷֳɸߡ���λ��������;
        {
            Scan_VH[i] = Scan_V[i] >> 8;
            Scan_VL[i] = Scan_V[i] & 0x00ff;
        }
        Usart_SendByte(&huart1, 0xEE);  //���ַ�
        for (int i = 0; i < 380; i++)
        {
            Usart_SendByte(&huart1, Scan_VH[i]);
            Usart_SendByte(&huart1, Scan_VL[i]);
        }
        Usart_SendByte(&huart1, 0xFF);    //β�ַ�
    }
    else ;
    /*****************************************************/
    LED_L;  //�ص�
}
/**
*   @brief:  ���߲��ԣ�
*   @param:  channel: ��ǰѡ��Ĳ���ͨ��
*   @return: void
*   @author Hang
*   @date: 
*/

void Graph_test(u8 channel)    //���߼��
{
    //item = 1;    //��ʱȡ���ж�����
    //Delete_screen_id07();    //������Խ���
    LED_H;
    HAL_Delay(800);  //����
    Motor_init(); //
//    Moter_EnOk();
    HAL_Delay(1000);    //1s
//    moter_Zero();                                                           //��ǰ��λ���
//    KaMoter_Step(Moter_speed, equipment_parament_2);                //���е� ��ֽ����ⴰ ���
//    Moter_MoveToChannel(channel, 0);
    Scan_win();   //ɨ�������Լ��Ӵ�,ȡ�� 380 ���������;
//    Moter_DropCard();  //ж�����������
//    Ka_InitBack(Moter_speed);
//    HAL_Delay(100);        //0.1s
//    Ka_InitFont(Moter_speed);
//    HAL_Delay(500);    //0.5s
//    Moter_EnNo();
    LED_L;  //�ص�
    for (int i = 0; i < 380; i++) ////��7������С����,�˴������������С100����ʾ;
    {
        //Scan_VH[i] = Scan_V[i]/100;
        Scan_VH[i] = Scan_V[i] >> 8;
        Scan_VL[i] = Scan_V[i] & 0x00ff;
    }

    //���ͻ�����������ָ�����ַ���     EE B1 32 00 07 00 01 00 02 00
//  Usart_SendByte(USART2,0xEE);Usart_SendByte(USART2,0xB1);Usart_SendByte(USART2,0x32);Usart_SendByte(USART2,0x00);Usart_SendByte(USART2,0x07);
//  Usart_SendByte(USART2,0x00);Usart_SendByte(USART2,0x01);Usart_SendByte(USART2,0x00);Usart_SendByte(USART2,0x02);Usart_SendByte(USART2,0x00);
    send_front();
    Usart1_SendByte(0x07);
    for (u32 i = 0; i < 380; i++) //��7������С����,�˴��ɲ����ͼ�����ݵ�λ;
    {
        Usart1_SendByte( Scan_VH[i]);
        Usart1_SendByte( Scan_VL[i]);
    }
    send_end();
    //Show_ending();   //��Ļ��ʾͨѶ��β


    HAL_Delay(100);    //100ms
//    switch (item)
//    {
//    case CRP:   //1   �ݶ�����Ŀ��2���ߣ�1�������
//        ResultCount = 1;
//        Graph_Result(ResultCount);
//        break;

//    case PG1:   //2  �ݶ�����Ŀ��3���ߣ�2�������
//        ResultCount = 2;
//        Graph_Result(ResultCount);
//        break;

//    case PG2:   //3  �ݶ�����Ŀ��4���ߣ�3�������
//        ResultCount = 3;
//        Graph_Result(ResultCount);
//        break;

//    default:
//        state = 0; //δ��⵽�Լ���
//        break;
//    }
}

/**
*   @brief: ��ʱ&��ʱ����
*   @param:  flag��00->��ʱ����; 01->��������
*   @return: void
*   @author Hang
*   @date:
*/

void Immediately_Test(u8 channel,u8 testFlag,u8 barcodeFlag)
{
    //Delete_Warning();
    if (testFlag == 0x00)
    {
        for (u16 a = 0; a < IncubationTime; a++)
        {
            HAL_Delay(1000);   //��ʱ����ʱ��
        }
    }
    //Delete_CodeNum();   // ���� ���� ����
    if (barcodeFlag == 0x01) //�����
    {
//     Moter_EnOk();
        HAL_Delay(300);
        //Save_DetectionTime();
//     KaMoter_Step(Moter_speed,2000);   //��ǰ����һ�ξ���
//     HAL_Delay(100);
//     Ka_InitFont(Moter_speed);                 //ǰ��Ѱ��,��λ;
        Motor_init();
//        Moter_MoveToChannel(channel, 1); //���е�ɨ���ѹ�Ӵ�λ��
//     Set_Form_Parameter();                  //����ַ�����ݱ��浽�β��У���ʱʹ��
//     show_screen_id01_ephemeraldata();      //��ʾ��ʱ����(��ˮ�ŵ�)
        Test_TC(channel,barcodeFlag);
//     Address_Zero();//���ø�����Ϣ�洢��ַ���׵�ַ Ϊ0
//     Moter_EnNo();
		HAL_Delay(200);
    }
    //else  if(Code_flag == 0x00)   //���뿪
//   {
//     //�˴��ж������Ƿ�ɨ��ɹ������ж��Ƿ���ID����Ϣ��
//     Code_Parameter[0] = 0x00;
//     //item = 0;                        //��ʼ����������ϢΪ��
//     Open_Scanning();   //����ɨ��ǹ
////     Moter_EnOk();
////     HAL_Delay(300);
////     KaMoter_Step(Moter_speed,equipment_parament_1);   //���е�ɨ������λ��
//     Moter_MoveToChannel(R2,0);//���е�ɨ������λ��
//      HAL_Delay(100);
//        Close_Scanning();
////     Ka_InitFont(Moter_speed);                 //ǰ��Ѱ��
//     //Save_CodeData(item);
//     if(item==0x01|item == 0x02|item==0x03)           //ɨ�赽������Ϣ
//        {
//          Set_Form_Parameter();                  //����ַ�����ݱ��浽�β��У���ʱʹ��
//          show_screen_id01_ephemeraldata();      //��ʾ��ʱ����(��ˮ�ŵ�)
//                  HAL_Delay(50);
//          Test_TC();
//                  //HAL_Delay(100);
//          Address_Zero();//���ø�����Ϣ�洢��ַ���׵�ַ Ϊ0
//          //Delete_SampleNum();
//          Moter_EnNo();
//        }
//     else if(item == 0x00)    //δɨ�赽������Ϣ
//        {
//          //Delete_screen_id01();    //������Խ���
//          Show_Waring01();
//                  Set_Form_Parameter();                  //����ַ�����ݱ��浽�β��У���ʱʹ��
//          //show_screen_id01_ephemeraldata();      //��ʾ��ʱ����(��ˮ�ŵ�)
//                  HAL_Delay(100);
//                  if(flag_tt == 1)
//                  {
//                      item = 1;
//                      Test_TC();
//                      item = 0;
//                      //HAL_Delay(100);
//                      Address_Zero();//���ø�����Ϣ�洢��ַ���׵�ַ Ϊ0
//                      //Delete_SampleNum();
//                      Moter_EnNo();
//                  }
//                  else
//                  {
//                      Moter_EnOk();Ka_InitFont(Moter_speed);
//            HAL_Delay(300); Moter_EnNo();LED_L;
//                  }
//        }
//   }
}

void getPosition()
{
//    send_front();
//    Usart_SendByte(&huart3, 0x0c);
//    Usart_SendByte(&huart3,equipment_addres_5);
//    Usart_SendByte(&huart3, equipment_addres_5 + 1);
//	
//    Usart_SendByte(&huart3, equipment_addres_6);
//    Usart_SendByte(&huart3, equipment_addres_6 + 1);
//    send_end();
	equipment_parament_5 = I2C_get2Byte(equipment_addres_5);
	equipment_parament_6 = I2C_get2Byte(equipment_addres_6);
	printf("ɨ�봰λ��%d",equipment_parament_5);
	printf("��ⴰλ��%d",equipment_parament_6);
}


void setPosition(u8 n)
{
    u16 params = (R3 - 0x30) * 10000 + (R4 - 0x30) * 1000 + (R5 - 0x30) * 100 + (R6 - 0x30) * 10 + (R7 - 0x30);
    if (n == 0x05)
    {
        I2C_write2Byte(equipment_addres_5, params);
		 equipment_parament_5 = I2C_get2Byte(equipment_addres_5);
    }
    else if (n == 0x06)
    {
        I2C_write2Byte(equipment_addres_6, params);
		equipment_parament_6 = I2C_get2Byte(equipment_addres_6);
    }
}

////////////////��PC��ͨѶ,��һ�ε�ѹֵ
void Scan_AD(void)
{
    u8 a1, a2, a3, a4, a5, a6, a7, a8;
    int b1;
    unsigned char c1, c2, c3, c4, c5;
    float V[10];      //��10��,ȡ��ֵ;
    u32 vvv;
    for (u8 i = 0; i < 10; i++)
    {
        V[i] = (float)ADC_ConvertedValue / 4096 * 3.3;
    }
    ADC_ConvertedValueLocal = V[0] + V[1] + V[2] + V[3] + V[4] + V[5] + V[6] + V[7] + V[8] + V[9];
    vvv = (ADC_ConvertedValueLocal / 10) * 10000;

    //Show_front(0x06,0x01);
//     send_front();
//     Usart_SendByte(&huart1, 0x01);
//	 Usart_SendByte(&huart1, 0x02);
//	 Usart_SendByte(&huart1, 0x03);
	
    if ((vvv > 10000) || (vvv == 10000))
    {
//		printf("vvvv");
        b1 = vvv;
        a1 = b1 / 10000;
        a2 = b1 % 10000;
        a3 = a2 / 1000;
        a4 = a2 % 1000;
        a5 = a4 / 100;
        a6 = a4 % 100;
        a7 = a6 / 10;
        a8 = a6 % 10;
        c1 = a1 + 0x30;
        c2 = a3 + 0x30;
        c3 = a5 + 0x30;
        c4 = a7 + 0x30;
        c5 = a8 + 0x30;
		
		Usart1_SendByte(c1);
		Usart1_SendByte(c2);
		Usart1_SendByte(c3);
		Usart1_SendByte(c4);
		Usart1_SendByte(c5);
		Usart1_SendByte(0xEF);
//		USART_SEND();
    }
    else if ((vvv > 1000) || (vvv == 1000))
    {
		printf("111111");
        b1 = vvv;
        a1 = b1 / 1000;
        a2 = b1 % 1000;
        a3 = a2 / 100;
        a4 = a2 % 100;
        a5 = a4 / 10;
        a6 = a4 % 10;
        c1 = a1 + 0x30;
        c2 = a3 + 0x30;
        c3 = a5 + 0x30;
        c4 = a6 + 0x30;
        Usart1_SendByte( c1);
        Usart1_SendByte( c2);
        Usart1_SendByte( c3);
        Usart1_SendByte(c4);
    }
    else if ((vvv > 100) || (vvv == 100))
    {
		printf("2222");
        b1 = vvv;
        a1 = b1 / 100;
        a2 = b1 % 100;
        a3 = a2 / 10;
        a4 = a2 % 10;
        c1 = a1 + 0x30;
        c2 = a3 + 0x30;
        c3 = a4 + 0x30;
        Usart1_SendByte( c1);
        Usart1_SendByte( c2);
        Usart1_SendByte( c3);
    }
    else
    {
		printf("3333");
        if ((vvv > 10) || (vvv == 10))
        {
            b1 = vvv;
            a1 = b1 / 10;
            a2 = b1 % 10;
            c1 = a1 + 0x30;
            c2 = a2 + 0x30;
            Usart1_SendByte( c1);
            Usart1_SendByte( c2);
        }
        else
        {
            b1 = vvv;
            c1 = b1 + 0x30;
            Usart1_SendByte(c1);
        }
    }
    //Show_ending();         //��Ļ��ʾͨѶ��β
//    send_end();
	
//	Usart_Send(0xFF);
	HAL_Delay(50);
}

//void params


//���յ�����CAN�ʹ��ڵ�ָ����з�������
void  getPraVue()
{
    uint32_t params;
    params = (R3 - 0x30) * 10000 + (R4 - 0x30) * 1000 + (R5 - 0x30) * 100 + (R6 - 0x30) * 10 + (R7 - 0x30);
    switch (R0)
    {
		case 0xB1:                      //0xB1:  ��������
        switch (R1)
        {
        case 0x01://X���ʼ��
            Ka2_InitBack(45);
            break;
        case 0x02://Y���ʼ��
//            Ka1_InitBack(45);
            break;
        case 0x03://������ʼ��
            Motor_init();
            break;
        case 0x04://��ȡ����λ�úͼ�ⴰλ��
            getPosition();
            break;
        case 0x05:// �趨ɨ�������
            setPosition(R1);
            break;
        case 0x06:// �趨��ⴰ����
            setPosition(R1);
            break;
        case 0x07://  ���е�����λ
//            Moter_MoveToChannel(R2, 0);
            break;
        case 0x08:// ���е���ⴰλ
//            Moter_MoveToChannel(R2, 1);
            break;
        case 0x09://  ��ȡ��ѹֵ
            Scan_AD();
            break;
        case 0x0A://����
            LED_H;
            break;
        case 0x0B://�ص�
            LED_L;
            break;
        case 0x11:// X�ᶨλ�ƶ�
            Ka2Moter_Step(R2, params);
		params++;
		printf("xmove�ľ���Ϊ%d",params);
            break;
        case 0x12://Y�ᶨλ�ƶ�
            Ka1Moter_Step(R2, params);
		params--;
		printf("ymove�ľ���Ϊ%d",params);
			break;
		case 0x13://ִ��ж������
//			Moter_DropCard();
            break;
		case 0x14: //��ȡ�ӽ�����״̬
//			if(HAL_GPIO_ReadPin(IN4_GPIO_Port,IN4_Pin) == KEY_OFF){
//				printf("ת��δ��λ");
//			}else{
//				printf("ת�̸�λ�Ѹ�λ");
//			
//			}
//				if((HAL_GPIO_ReadPin(IN5_GPIO_Port,IN5_Pin) == KEY_ON)){
//				printf("���δ��λ");
//			}else{
//			   printf("����Ѹ�λ");
//			}
			Motor_init();
			break;
		case 0x15: // ת�̸�λ����
			Ka2_InitBack2(1,params);
			break;
		case 0x16: //ת���ƶ�
			Ka2Moter_Step(1,params);
			break;
		case 0x17: // ���Ե������ิλ
			Ka1_InitBack(1,0);   
			break;
		case 0x18: // ���Ե�����ڲ��ƶ�
			Ka1Moter_Step(1,params);
			break;
		case 0x19: // �˿�������λ
			Ka3_InitBack(1);   // �˿������λ
			break;
		case 0x20: // �˿�������λ ��ǰж���ƶ�
			Ka3Moter_Step(1,params);
			break;
		case 0x21: // �ӿ�����
			addCard();
			break;
		case 0x22: // ж������
			dropCard();
			break;
		case 0x23: //��������
			testStatus=1;
			break;
		case 0x24: //�رղ���
			testStatus=0;
			break;
		case 0x25: //ж���Ӽ��ٲ���
			REF3_H;	
			EN3_H; 
			printf("�ƶ��ľ���Ϊ��%d",params);
			moter3_step(params);
//			EN3_L; 
			break;
			case 0x27: //ֹͣʹ��
			EN3_L; 
			break;
		case 0x26: //ж���Ӽ��ٲ���
			addCard();
			break;
		case 0x28: //ж�������λ
			REF3_H;	
			EN3_H; 
			moter3_int();
			break;
		case 0x29: //ж�������λ
		if(HAL_GPIO_ReadPin(IN4_GPIO_Port,IN4_Pin) == KEY_OFF)
		{
			printf("false");
		}else{
			printf("true");
		}
			break;
		case 0x30: //I2C ��д����
			Read_equipment_parament();
			break;
		case 0x31: //�ƶ�ָ�����ۺŵ�����λ��
			testCard(R2);
			break;
		case 0x32: //����ж����λ
			dropCard();
			break;
        default:
            break;
        }
        break;
    case 0xA1:
        switch (R1)
        {
        case 01://PC Test
            PC_test();                     //���ͼ�����ݸ�PC��
            break;
        case 02: //��ʱ Test
            Immediately_Test(R2,R3,R4);
            break;
        case 03: // ���߲���
            Graph_test(R2);
            break;
		case 04:// ��ȡ��ǰ���۲忨״̬:
			Scan_card();
        default:
            break;
        }
			HAL_Delay(500);
            send_front();
            send_end();
        break;
    default:
        break;
    }
}

//�����жϵ�ǰ�Ƿ񻹴��ڲ忨ģʽ
 void isWaitCard(){
	  switch (R0)
    {	
    case 0xB2:
        switch (R1)
        {
        case 01://
            testStatus=1;            
            break;
		 case 02://
            testStatus=0;            
            break;
        default:
            break;
        }
        break;
    default:
        break;
    }
};


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
			
			isWaitCard();
//			HAL_Delay(50);
//			if(CurUartNum==1){
//				rxbuf[RxLine++]=0xAA;
//				HAL_UART_Transmit(&huart1, rxbuf, RxLine , 1000);
//				HAL_UART_Transmit(&huart3, rxbuf, RxLine, 1000);
//			}else if(CurUartNum==2){
//				rxbuf[RxLine++]=0xAA;
//				HAL_UART_Transmit(&huart3, rxbuf, RxLine, 1000);
//				HAL_UART_Transmit(&huart1, rxbuf, RxLine, 1000);
//			}
		
            getPraVue();
//			HAL_Delay(50);
//			if(CurUartNum==1){
//				rxbuf[RxLine++]=0xBB;
//				HAL_UART_Transmit(&huart1, rxbuf, RxLine, 1000);
//				HAL_UART_Transmit(&huart3, rxbuf, RxLine, 1000);
//			}else if(CurUartNum==2){
//				rxbuf[RxLine++]=0xBB;
//				HAL_UART_Transmit(&huart3, rxbuf, RxLine, 1000);
//				HAL_UART_Transmit(&huart1, rxbuf, RxLine, 1000);
//			}
//			HAL_Delay(50);
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
            }
            HAL_UART_Transmit(&huart1, CanRXmessage, sizeof(CanRXmessage), 1000);
            CAN_senddata(&hcan, TextBuffer);
            Rx_end = 0;
            getPraVue();
        }
}




//��ʱ���ص�����
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    /* USER CODE BEGIN Callback 0 */
    static uint32_t num = 0;
    /* USER CODE END Callback 0 */
    if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
    /* USER CODE BEGIN Callback 1 */
    else if (htim->Instance == TIM2) // ��ʱ��2����ַ
    {
        // �Զ���Ӧ�ó���
        time++;
        num++;
        if (num == 100000) // ÿ1��LED�Ʒ�תһ��
        {
//            HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0);
//            CAN_senddata(&hcan, TextBuffer);
            num = 0;
        }
    }
	else if(htim->Instance==TIM3){  //���Ƶ��PWM���
		//����Ӽ���ʵ�ֹ��ܡ�
//		TIM3Fun();
//		printf("motor �Ӽ��ٲ���");
	}	else if(htim->Instance==TIM4){  //���Ƶ��PWM���
		//����Ӽ���ʵ�ֹ��ܡ�
		TIM4Fun();
//		printf("motor �Ӽ��ٲ���");
	}
	
    /* USER CODE END Callback 1 */
}
