#include "work_interface.h"
#include "eeprom.h"

#include "can.h"
#include "i2c.h"
//#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "motor.h"

#include "S_Moter1.h"  


//测试变量
uint32_t  getNumber;

//中断计时变量
uint32_t time;

uint8_t R0, R1, R2, R3, R4, R5, R6, R7, R8, R9; //用来接收串口下发的指令。
//串口通讯变量
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;

//当前使用的串口位；
extern uint8_t CurUartNum;

extern uint8_t  RecieveBuffer[1];//暂存接收到的字符
extern uint8_t  Rx_end;  //指令接收完成标志位
extern uint8_t  RxLine;  //rxbuf接收的数据长度为:RxLine+1;
extern uint8_t  rxbuf[50];//收到的数据存放处
uint8_t TextBuffer[8] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x44, 0x55, 0x55};

////CAN通讯变量：
extern CAN_TxHeaderTypeDef TXHeader;
extern CAN_RxHeaderTypeDef RXHeader;
extern uint8_t CanTXmessage[8];
extern uint8_t CanRXmessage[8];
extern uint32_t pTxMailbox;



//Can发送数据函数：
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

// ADC转换值
__IO uint32_t ADC_ConvertedValue;
// 用于保存转换计算后的电压值
float ADC_Vol;


/*
*  *********************************检测使用的中间变量*******************************
*/
u32 moter3_now;      //Z轴电机当前位置
u32 moter3_next;     //Z轴电机当前位置

u8 motor3Flag; //  Z轴电机复位标志位

u32 moter1_now;      //X轴电机当前位置
u32 moter1_next;     //X轴电机当前位置


//当前被插入试剂的卡槽数
uint8_t NumberOfCards;
//当前机器状态
uint8_t testStatus;
//当前测试的试剂的卡槽序号
uint8_t testIndex;

//转盘移动的初始距离
uint8_t StartStep;

//转盘移动的间隔距离
uint8_t SingleStep;

//卡槽状态标志位  1：有卡 0：无卡
uint8_t card1_flag;
uint8_t card2_flag;
uint8_t card3_flag;
uint8_t card4_flag;
uint8_t card5_flag;
uint8_t card6_flag;


uint16_t Ka_now;  //检测镜头当前位置
uint16_t Ka_next;//检测镜头下一步位置

u32 Scan_V[380];     //检测的原始数据
u32 Scan_VH[380];    //原始数据高位
u32 Scan_VL[380];    //原始数据低位
float Scan_Float_V[380];

float ADC_ConvertedValueLocal;

float Result_1;                      //标记检测结果
float Result_2;                      //标记检测结果
float Result_3;                      //标记检测结果
u8 state = 0;  //判断是否检测到带条码的试剂卡进入
u8 ResultCount;                 //检测结果的数量: 两条线 ResultCount=1; 三条线 ResultCount=2; 四条线 ResultCount=3;
extern u8 UART_Code[7]; //条码值
//////////////////////////////////////////////
//发送帧头
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
//发送帧尾
/////////////////////////
void send_end()
{
	USART_SendByte(0xFF);
	USART_SendByte(0xFE);
	
//    Usart1_SendByte(0xFF);
//    Usart1_SendByte(0xFE);
}



void Open_Scanning(void)//打开条码器
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

void Close_Scanning(void)//关闭条码器
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

    检测复位状态
*/
void rd_io()
{
    u8 step1 = 30;
    u8 step2 = 30;
//  HAL_GPIO_ReadPin(IN0_GPIO_Port,IN0_Pin)
    if (HAL_GPIO_ReadPin(IN1_GPIO_Port, IN1_Pin) == KEY_OFF) //判断测试轨道方向是否复位
    {
        EN1_H;
        DIR1_L;
//      GENERAL_TIM_Init();                         /*定时器初始化*/
//      TIM_Cmd(GENERAL_TIM, DISABLE);              //关闭定时器
//      TIM_Cmd(GENERAL_TIM, ENABLE);               //开启定时器
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
    if (HAL_GPIO_ReadPin(IN1_GPIO_Port, IN1_Pin) == KEY_OFF) //判断换道轨道方向是否复位
    {
        EN2_H;
        DIR2_L;
//      GENERAL_TIM_Init();                         /*定时器初始化*/
//      TIM_Cmd(GENERAL_TIM, DISABLE);              //关闭定时器
//      TIM_Cmd(GENERAL_TIM, ENABLE);               //开启定时器
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
		printf("卡槽1有卡");
    }
    else{
	
        card6_flag = 0;
		printf("卡槽1无卡");
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

//参数初始化
void init()
{
//	HAL_Delay(35000);
//    Scan_card();
	printf("system init");
    Motor_init();//电机复位
    Read_equipment_parament(); //获取电机参数
	printf("system init");
}

void Read_AD_All(unsigned int mmm)
{
    float V[100];      //读100次,取均值;
    float max = 0, min = 0;
    /*  增加单次采点次数并取平均值  */
    for (u8 i = 0; i < 100; i++)
    {
        V[i] = (float) ADC_ConvertedValue / 4096 * 3.3;
    }
    for (u8 a = 0; a < 100; a++)
    {
        if (max > V[a])
        {
            max = V[a];   //取出一个最高值
        }
    }
    for (u8 b = 0; b < 100; b++)
    {
        if (min < V[b])
        {
            min = V[b];   //取出一个最低值
        }
    }

    ADC_ConvertedValueLocal = 0;
    for (u8 i = 0; i < 100; i++)
    {
        ADC_ConvertedValueLocal = ADC_ConvertedValueLocal + V[i];
    }
    ADC_ConvertedValueLocal = ADC_ConvertedValueLocal - max - min;  //减掉最高值和最低值
    Scan_V[mmm] = (ADC_ConvertedValueLocal / 98) * 10000;
    Scan_Float_V[mmm] = ADC_ConvertedValueLocal / 98;
}




void Scan_win()   //扫描整个试剂调显色窗   flag == 1: PC检测,发送762个数据给PC机;
//                      flag == 2: 屏幕检测;
{
    u8 fit_num = 10;     //设定拟合次数
//    float max = 0, min = 0;
	EN1_H;
    for (int i = 0; i < 380; i++)  //滑块每运行 4 步 , 进行 1 次检测;
    {
        //圆柱轨道 每运行 6步
//        KaMoter_Step(Moter_speed + 20, X_step + 4 * i);
//        Ka1Moter_RunStep(1, equipment_parament_6 - 23* i);  //得被替换2023-0331
        Read_AD_All(i);
    }
//	EN1_H;
    /***************** 此处进行曲线滤波处理     5 点 取均值 前后各取5点    *****************/
    for (u8 j = 0; j < fit_num; j++) //经过10次拟合,曲线平滑度较好;
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

    for (u8 j = 0; j < fit_num; j++) //经过10次拟合,曲线平滑度较好;
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


//void Test_Result(u8 num)    //num:该项目有几个检测结果;
//{
//  u32 Area;                                  //取值范围
//  u32 t[8]={0};                              //标记各个峰值起、终点位置
//  u32 MaxX[4]={0};                          //标记各个峰值的X坐标
//  float MaxV[4]={0};                          //标记各个峰值数据
//  float MaxArea[4]={0};                       //标记各个峰值面积数据
//  float MinArea[4]={0};                       //标记各个峰值面积的本底数据
//  float a,b,c,d;
//
//  switch(num)
//    {
//      case 1:   //一个检测结果,两条峰值(MaxV1,MaxV2);
//        t[0] = equipment_parament_3;                  //第一个峰值的起点
//        t[1] = equipment_parament_4;                  //第一个峰值的终点
//        t[2] = equipment_parament_4+1;                //第二个峰值的起点
//        t[3] = equipment_parament_5;                  //第二个峰值的终点
//        Area = equipment_parament_21;
//
//        MaxV[0] = Scan_Float_V[t[0]];
//        for(u16 i=t[0];i<t[1];i++)
//          {
//            if(Scan_Float_V[i] > MaxV[0])
//              {MaxV[0] = Scan_Float_V[i]; MaxX[0] = i;}  //第一个峰值 以及 第一个峰值的X坐标
//          }
//        MaxV[1] = Scan_Float_V[t[2]];
//        for(u16 j=t[2];j<t[3];j++)
//          {
//            if(Scan_Float_V[j] > MaxV[1])
//              {MaxV[1] = Scan_Float_V[j]; MaxX[1] = j;}  //第二个峰值 以及 第二个峰值的X坐标
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
////        Show_ItemName(0x01,0x12);           //显示 子项目名
////                HAL_Delay(50);
////        Show_Result(0x01,0x13,Result_1);    //显示结果1     T1/C
//        HAL_Delay(50);
//        //Show_unit(0x01,0x14);      //显示单位
//      break;
//
//      case 2:               //二个检测结果,三条峰值(MaxV1,MaxV2,MaxV3);
//        t[0] = CurvePeak_Start1;                  //第一个峰值的起点0
//        t[1] = CurvePeak_End1;                  //第一个峰值的终点100
//        t[2] = CurvePeak_Start2;                //第二个峰值的起点100
//        t[3] = CurvePeak_End2;                  //第二个峰值的终点200
//        t[4] = CurvePeak_Start3;                //第三个峰值的起点200
//        t[5] = CurvePeak_End3;                  //第三个峰值的终点255
//        Area = referenceValue;
//
//        MaxV[0] = Scan_Float_V[t[0]];
//        for(u16 i=t[0];i<t[1];i++)
//          {
//            if(Scan_Float_V[i]-MaxV[0])
//              {MaxV[0] = Scan_Float_V[i]; MaxX[0] = i;}  //第一个峰值 以及 第一个峰值的X坐标
//          }
//        MaxV[1] = Scan_Float_V[t[2]];
//        for(u16 j=t[2];j<t[3];j++)
//          {
//            if(Scan_Float_V[j] > MaxV[1])
//              {MaxV[1] = Scan_Float_V[j]; MaxX[1] = j;}  //第二个峰值 以及 第二个峰值的X坐标
//          }
//        MaxV[2] = Scan_Float_V[t[4]];
//        for(u16 k=t[4];k<t[5];k++)
//          {
//            if(Scan_Float_V[k] > MaxV[2])
//              {MaxV[2] = Scan_Float_V[k]; MaxX[2] = k;}  //第三个峰值 以及 第三个峰值的X坐标
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
////        Show_ItemName(0x01,0x12);           //显示 子项目名
////                HAL_Delay(50);
////        Show_Result(0x01,0x13,Result_1);    //显示结果1     T1/C
//              HAL_Delay(50);
//        //Show_unit(0x01,0x14);               //显示单位
//
////        Show_ItemName(0x01,0x15);           //显示 子项目名
////                HAL_Delay(50);
////        Show_Result(0x01,0x16,Result_2);    //显示结果1     T2/C
//              HAL_Delay(50);
//        //Show_unit(0x01,0x17);               //显示单位
//      break;

//      case 3:                       //三个检测结果,三条峰值(MaxV1,MaxV2,MaxV3,MaxV4);
//        t[0] = equipment_parament_10;                  //第一个峰值的起点
//        t[1] = equipment_parament_11;                  //第一个峰值的终点
//        t[2] = equipment_parament_11+1;                //第二个峰值的起点
//        t[3] = equipment_parament_12;                  //第二个峰值的终点
//        t[4] = equipment_parament_12+1;                //第三个峰值的起点
//        t[5] = equipment_parament_13;                  //第三个峰值的终点
//        t[6] = equipment_parament_13+1;                //第四个峰值的起点
//        t[7] = equipment_parament_14;                  //第四个峰值的终点
//        Area = equipment_parament_23;
//
//        MaxV[0] = Scan_Float_V[t[0]];
//        for(u16 i=t[0];i<t[1];i++)
//          {
//            if(Scan_Float_V[i] > MaxV[0]){MaxV[0] = Scan_Float_V[i]; MaxX[0] = i;}  //第一个峰值 以及 第一个峰值的X坐标
//          }
//        MaxV[1] = Scan_Float_V[t[2]];
//        for(u16 j=t[2];j<t[3];j++)
//          {
//            if(Scan_Float_V[j] > MaxV[1]){MaxV[1] = Scan_Float_V[j]; MaxX[1] = j;}  //第二个峰值 以及 第二个峰值的X坐标
//          }
//        MaxV[2] = Scan_Float_V[t[4]];
//        for(u16 k=t[4];k<t[5];k++)
//          {
//            if(Scan_Float_V[k] > MaxV[2]){MaxV[2] = Scan_Float_V[k]; MaxX[2] = k;}  //第三个峰值 以及 第三个峰值的X坐标
//          }
//        MaxV[3] = Scan_Float_V[t[6]];
//        for(u16 l=t[6];l<t[7];l++)
//          {
//            if(Scan_Float_V[l] > MaxV[3]){MaxV[3] = Scan_Float_V[l]; MaxX[3] = l;}  //第三个峰值 以及 第三个峰值的X坐标
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
////        Show_ItemName(0x01,0x12);           //显示 子项目名
////                HAL_Delay(50);
////        Show_Result(0x01,0x13,Result_1);    //显示结果1     T1/C
//              HAL_Delay(50);
//        //Show_unit(0x01,0x14);               //显示单位
//
//        Show_ItemName(0x01,0x15);           //显示 子项目名
//              HAL_Delay(50);
//        Show_Result(0x01,0x16,Result_2);    //显示结果1     T2/C
//              HAL_Delay(50);
//        //Show_unit(0x01,0x17);               //显示单位
//
//        Show_ItemName(0x01,0x18);           //显示 子项目名
//              HAL_Delay(50);
//        Show_Result(0x01,0x19,Result_3);    //显示结果2     T3/C
//              HAL_Delay(50);
//        //Show_unit(0x01,0x1A);               //显示单位
//      break;
//      default:     break;
//    }
//      Save_Data(num);  //num 标记共有几个检测结果
//}

void Test_TC(u8 n,u8 flag)    //扫描试剂卡 视窗
{
//    u8 SampleNum_H, SampleNum_L;
    HAL_Delay(500);    //0.5s
//    KaMoter_Step(Moter_speed, equipment_parament_2);  //运行到 试纸条检测窗 起点
//    Moter_MoveToChannel(n, 1);//
    LED_H;   //开灯
    HAL_Delay(800);   //等待灯源能量稳定  时间不小于700ms
    Scan_win();   //扫描整个试剂视窗,取得 380 个点的数据;
    HAL_Delay(500);     //0.05s
    LED_L;  //关灯
//    Moter_DropCard();//
    HAL_Delay(500);    //0.5s
//    Ka_InitBack(Moter_speed); //试剂卡向后复位 ,卸载试剂卡;


//    Ka_InitFont(Moter_speed); //试剂卡向前复位;
    //item = 1;//配合条码测试，调试完成时，该值去掉

	Code_flag=flag;
    if (Code_flag == 0x01)      //条码器关闭的情况，默认两个峰值;
    {
        state = 1; //检测到试剂卡
        ResultCount = 1;
//        Test_Result(ResultCount);  这个是计算峰值比较值。结果现在是放在android屏幕实现。
        for (int i = 0; i < 380; i++) ////受7寸屏大小限制,此处将检测数据缩小100倍显示;
        {
            //Scan_VH[i] = Scan_V[i]/100;
            Scan_VH[i] = Scan_V[i] >> 8;
            Scan_VL[i] = Scan_V[i] & 0x00ff;
        }

        //发送绘制正旋曲线指令首字符组     EE B1 32 00 07 00 01 00 02 00
        send_front();
//        Usart_SendByte(&huart1, 0x07);
		USART_SendByte(0x07);
        for (u32 i = 0; i < 380; i++) //受7寸屏大小限制,此处可不发送检测数据低位;
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
//  else if (Code_flag == 0x00)  //条码器打开的情况
//    {
//        switch (item)
//        {
//        case CRP:   //1   暂定该项目有2条线，1个检测结果
//            state = 1; //检测到试剂卡
//            ResultCount = 1;
//            Test_Result(ResultCount);
//            for (int i = 0; i < 380; i++) ////受7寸屏大小限制,此处将检测数据缩小100倍显示;
//            {
//                //Scan_VH[i] = Scan_V[i]/100;
//                Scan_VH[i] = Scan_V[i] >> 8;
//                Scan_VL[i] = Scan_V[i] & 0x00ff;
//            }
//            //发送绘制正旋曲线指令首字符组     EE B1 32 00 07 00 01 00 02 00
//            send_front();
//            Usart_SendByte(&huart3, 0x07);
//            for (u32 i = 0; i < 380; i++) //受7寸屏大小限制,此处可不发送检测数据低位;
//            {
//                Usart_SendByte(&huart3, Scan_VH[i]);
//                Usart_SendByte(&huart3, Scan_VL[i]);
//            }
//            send_end();
//            break;
//        case PG1:   //2  暂定该项目有3条线，2个检测结果
//            state = 1; //检测到试剂卡
//            ResultCount = 2;
//            Test_Result(ResultCount);
//            break;
//        case PG2:   //3  暂定该项目有4条线，3个检测结果
//            state = 1; //检测到试剂卡
//            ResultCount = 3;
//            Test_Result(ResultCount);
//            break;
//        default:
//            item = 0;
//            state = 0; //未检测到试剂卡
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
//            HAL_Delay(1000);   //延时孵育时间
//        }
//    }
//    Code_flag = I2C_get2Byte(Code_flag_address);  //读取条码开关状态
//    if (Code_flag == 0x00)  //条码开
//    {
//        //此处判读条码是否扫描成功，并判断是否有ID卡信息；
//        Code_Parameter[0] = 0x00;
//        //item = 0;                        //初始设置条码信息为零
//        Open_Scanning();                                   //
//        HAL_Delay(300);
//        KaMoter_Step(Moter_speed, equipment_parament_1);  //运行到扫描条码位置
//        HAL_Delay(100);
//        Close_Scanning();
//        Ka_InitFont(Moter_speed);                 //前端寻零
//        //Save_CodeData(item);

//        if (item == 0x01 | item == 0x02 | item == 0x03)  //扫描到条码信息
//        {
//            Set_Form_Parameter();                  //将地址内数据保存到形参中，临时使用
//            show_screen_id01_ephemeraldata();      //显示临时数据(流水号等)
//            HAL_Delay(50);
//            Test_TC();
//            //HAL_Delay(100);
//            Address_Zero();//设置各类信息存储地址的首地址 为0
//            //Delete_SampleNum();
//            Moter_EnNo();
//        }
//        else if (item == 0x00)   //未扫描到条码信息
//        {
//            //Delete_screen_id01();    //清除测试界面
//            Show_Waring01();
//            Set_Form_Parameter();                  //将地址内数据保存到形参中，临时使用
//            //show_screen_id01_ephemeraldata();      //显示临时数据(流水号等)
//            HAL_Delay(100);
//            if (flag_tt == 1)
//            {
//                item = 1;
//                Test_TC();
//                item = 0;
//                //HAL_Delay(100);
//                Address_Zero();//设置各类信息存储地址的首地址 为0
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
//    else if (Code_flag == 0x01) //条码关
//    {
//        Moter_EnOk();
//        HAL_Delay(300);
//        //Save_DetectionTime();

//        KaMoter_Step(Moter_speed, 2000);  //向前运行一段距离
//        HAL_Delay(100);
//        Ka_InitFont(Moter_speed);                 //前端寻零,定位;

//        Set_Form_Parameter();                  //将地址内数据保存到形参中，临时使用
//        show_screen_id01_ephemeraldata();      //显示临时数据(流水号等)
//        Test_TC();
//        Address_Zero();//设置各类信息存储地址的首地址 为0
//        Moter_EnNo();
//    }
//}



void PC_test(void)    //PC机检测
{
    float PC_data[380];

    Motor_init();//电机初始化
//    Moter_EnOk();     //电机使能
    HAL_Delay(500);   //5s 等待灯源能量稳定
//    moter_Zero();                                                           //向前定位零点
//    KaMoter_Step(Moter_speed, equipment_parament_2);              //运行到 试纸条检测窗 起点
    Ka2Moter_Step(MotorY_speed, equipment_parament_6);          //运行到 试纸条检测窗 起点
//  Motor
    u8 Channel = R4; //获取测试通道位
//    Moter_MoveToChannel(Channel, 0);
    LED_H;    //开灯
    HAL_Delay(800);   //等待灯源能量稳定  时间不小于700ms
    Scan_win();   //扫描整个试剂视窗,取得 380 个点的数据;
    HAL_Delay(500);     //0.5s
    LED_L;  //关灯
    for (u32 i = 0; i < 380; i++)
    {
        PC_data[i] = (long double)Scan_V[i] * 0.0001;
    }

    if (R2 == 0x01) //判断是否卸载试剂卡   01:卸载;   00:不卸载;
    {
        HAL_Delay(100);    //0.1s
//        Ka_InitBack(Moter_speed);
//        Moter_DropCard();  //卸卡
    }
    HAL_Delay(100);        //0.1s
//    Ka_InitFont(Moter_speed);
    /****************打印出所有检测点电压值****************/
    if (R3 == 0x01) //判断数据发送类型   =0x01 :发送整数
    {
        //for(int i=0;i<380;i++)
        for (int i = 3; i < 377; i++)
        {
            HAL_Delay(10);   //0.01s
            //printf("%d \r\n",Scan_V[i]);
            printf("%f \r\n", PC_data[i]);
        }
    }
    /****************打印出所有检测点电压值****************/
    else if (R3 == 0x00)  //判断数据发送类型   =0x00 :发送高、低位共760 + 2 个数据
    {
        for (int i = 0; i < 380; i++) //一个数据分成高、低位两个数据;
        {
            Scan_VH[i] = Scan_V[i] >> 8;
            Scan_VL[i] = Scan_V[i] & 0x00ff;
        }
        Usart_SendByte(&huart1, 0xEE);  //首字符
        for (int i = 0; i < 380; i++)
        {
            Usart_SendByte(&huart1, Scan_VH[i]);
            Usart_SendByte(&huart1, Scan_VL[i]);
        }
        Usart_SendByte(&huart1, 0xFF);    //尾字符
    }
    else ;
    /*****************************************************/
    LED_L;  //关灯
}
/**
*   @brief:  曲线测试：
*   @param:  channel: 当前选择的测试通道
*   @return: void
*   @author Hang
*   @date: 
*/

void Graph_test(u8 channel)    //曲线检测
{
    //item = 1;    //临时取消判断条码
    //Delete_screen_id07();    //清除测试界面
    LED_H;
    HAL_Delay(800);  //开灯
    Motor_init(); //
//    Moter_EnOk();
    HAL_Delay(1000);    //1s
//    moter_Zero();                                                           //向前定位零点
//    KaMoter_Step(Moter_speed, equipment_parament_2);                //运行到 试纸条检测窗 起点
//    Moter_MoveToChannel(channel, 0);
    Scan_win();   //扫描整个试剂视窗,取得 380 个点的数据;
//    Moter_DropCard();  //卸卡、电机归零
//    Ka_InitBack(Moter_speed);
//    HAL_Delay(100);        //0.1s
//    Ka_InitFont(Moter_speed);
//    HAL_Delay(500);    //0.5s
//    Moter_EnNo();
    LED_L;  //关灯
    for (int i = 0; i < 380; i++) ////受7寸屏大小限制,此处将检测数据缩小100倍显示;
    {
        //Scan_VH[i] = Scan_V[i]/100;
        Scan_VH[i] = Scan_V[i] >> 8;
        Scan_VL[i] = Scan_V[i] & 0x00ff;
    }

    //发送绘制正旋曲线指令首字符组     EE B1 32 00 07 00 01 00 02 00
//  Usart_SendByte(USART2,0xEE);Usart_SendByte(USART2,0xB1);Usart_SendByte(USART2,0x32);Usart_SendByte(USART2,0x00);Usart_SendByte(USART2,0x07);
//  Usart_SendByte(USART2,0x00);Usart_SendByte(USART2,0x01);Usart_SendByte(USART2,0x00);Usart_SendByte(USART2,0x02);Usart_SendByte(USART2,0x00);
    send_front();
    Usart1_SendByte(0x07);
    for (u32 i = 0; i < 380; i++) //受7寸屏大小限制,此处可不发送检测数据低位;
    {
        Usart1_SendByte( Scan_VH[i]);
        Usart1_SendByte( Scan_VL[i]);
    }
    send_end();
    //Show_ending();   //屏幕显示通讯结尾


    HAL_Delay(100);    //100ms
//    switch (item)
//    {
//    case CRP:   //1   暂定该项目有2条线，1个检测结果
//        ResultCount = 1;
//        Graph_Result(ResultCount);
//        break;

//    case PG1:   //2  暂定该项目有3条线，2个检测结果
//        ResultCount = 2;
//        Graph_Result(ResultCount);
//        break;

//    case PG2:   //3  暂定该项目有4条线，3个检测结果
//        ResultCount = 3;
//        Graph_Result(ResultCount);
//        break;

//    default:
//        state = 0; //未检测到试剂卡
//        break;
//    }
}

/**
*   @brief: 即时&定时测试
*   @param:  flag：00->延时测试; 01->立即测试
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
            HAL_Delay(1000);   //延时孵育时间
        }
    }
    //Delete_CodeNum();   // 擦除 条码 数据
    if (barcodeFlag == 0x01) //条码关
    {
//     Moter_EnOk();
        HAL_Delay(300);
        //Save_DetectionTime();
//     KaMoter_Step(Moter_speed,2000);   //向前运行一段距离
//     HAL_Delay(100);
//     Ka_InitFont(Moter_speed);                 //前端寻零,定位;
        Motor_init();
//        Moter_MoveToChannel(channel, 1); //运行到扫描电压视窗位置
//     Set_Form_Parameter();                  //将地址内数据保存到形参中，临时使用
//     show_screen_id01_ephemeraldata();      //显示临时数据(流水号等)
        Test_TC(channel,barcodeFlag);
//     Address_Zero();//设置各类信息存储地址的首地址 为0
//     Moter_EnNo();
		HAL_Delay(200);
    }
    //else  if(Code_flag == 0x00)   //条码开
//   {
//     //此处判读条码是否扫描成功，并判断是否有ID卡信息；
//     Code_Parameter[0] = 0x00;
//     //item = 0;                        //初始设置条码信息为零
//     Open_Scanning();   //开启扫码枪
////     Moter_EnOk();
////     HAL_Delay(300);
////     KaMoter_Step(Moter_speed,equipment_parament_1);   //运行到扫描条码位置
//     Moter_MoveToChannel(R2,0);//运行到扫描条码位置
//      HAL_Delay(100);
//        Close_Scanning();
////     Ka_InitFont(Moter_speed);                 //前端寻零
//     //Save_CodeData(item);
//     if(item==0x01|item == 0x02|item==0x03)           //扫描到条码信息
//        {
//          Set_Form_Parameter();                  //将地址内数据保存到形参中，临时使用
//          show_screen_id01_ephemeraldata();      //显示临时数据(流水号等)
//                  HAL_Delay(50);
//          Test_TC();
//                  //HAL_Delay(100);
//          Address_Zero();//设置各类信息存储地址的首地址 为0
//          //Delete_SampleNum();
//          Moter_EnNo();
//        }
//     else if(item == 0x00)    //未扫描到条码信息
//        {
//          //Delete_screen_id01();    //清除测试界面
//          Show_Waring01();
//                  Set_Form_Parameter();                  //将地址内数据保存到形参中，临时使用
//          //show_screen_id01_ephemeraldata();      //显示临时数据(流水号等)
//                  HAL_Delay(100);
//                  if(flag_tt == 1)
//                  {
//                      item = 1;
//                      Test_TC();
//                      item = 0;
//                      //HAL_Delay(100);
//                      Address_Zero();//设置各类信息存储地址的首地址 为0
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
	printf("扫码窗位置%d",equipment_parament_5);
	printf("检测窗位置%d",equipment_parament_6);
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

////////////////与PC机通讯,读一次电压值
void Scan_AD(void)
{
    u8 a1, a2, a3, a4, a5, a6, a7, a8;
    int b1;
    unsigned char c1, c2, c3, c4, c5;
    float V[10];      //读10次,取均值;
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
    //Show_ending();         //屏幕显示通讯结尾
//    send_end();
	
//	Usart_Send(0xFF);
	HAL_Delay(50);
}

//void params


//对收到来自CAN和串口的指令进行分析处理
void  getPraVue()
{
    uint32_t params;
    params = (R3 - 0x30) * 10000 + (R4 - 0x30) * 1000 + (R5 - 0x30) * 100 + (R6 - 0x30) * 10 + (R7 - 0x30);
    switch (R0)
    {
		case 0xB1:                      //0xB1:  单步调试
        switch (R1)
        {
        case 0x01://X轴初始化
            Ka2_InitBack(45);
            break;
        case 0x02://Y轴初始化
//            Ka1_InitBack(45);
            break;
        case 0x03://仪器初始化
            Motor_init();
            break;
        case 0x04://获取条码位置和检测窗位置
            getPosition();
            break;
        case 0x05:// 设定扫条码距离
            setPosition(R1);
            break;
        case 0x06:// 设定检测窗距离
            setPosition(R1);
            break;
        case 0x07://  运行到条码位
//            Moter_MoveToChannel(R2, 0);
            break;
        case 0x08:// 运行到检测窗位
//            Moter_MoveToChannel(R2, 1);
            break;
        case 0x09://  读取电压值
            Scan_AD();
            break;
        case 0x0A://开灯
            LED_H;
            break;
        case 0x0B://关灯
            LED_L;
            break;
        case 0x11:// X轴定位移动
            Ka2Moter_Step(R2, params);
		params++;
		printf("xmove的距离为%d",params);
            break;
        case 0x12://Y轴定位移动
            Ka1Moter_Step(R2, params);
		params--;
		printf("ymove的距离为%d",params);
			break;
		case 0x13://执行卸卡操作
//			Moter_DropCard();
            break;
		case 0x14: //获取接近开关状态
//			if(HAL_GPIO_ReadPin(IN4_GPIO_Port,IN4_Pin) == KEY_OFF){
//				printf("转盘未复位");
//			}else{
//				printf("转盘复位已复位");
//			
//			}
//				if((HAL_GPIO_ReadPin(IN5_GPIO_Port,IN5_Pin) == KEY_ON)){
//				printf("检测未复位");
//			}else{
//			   printf("检测已复位");
//			}
			Motor_init();
			break;
		case 0x15: // 转盘复位距离
			Ka2_InitBack2(1,params);
			break;
		case 0x16: //转盘移动
			Ka2Moter_Step(1,params);
			break;
		case 0x17: // 测试电机朝外侧复位
			Ka1_InitBack(1,0);   
			break;
		case 0x18: // 测试电机朝内侧移动
			Ka1Moter_Step(1,params);
			break;
		case 0x19: // 退卡电机向后复位
			Ka3_InitBack(1);   // 退卡电机复位
			break;
		case 0x20: // 退卡电机向后复位 向前卸卡移动
			Ka3Moter_Step(1,params);
			break;
		case 0x21: // 加卡测试
			addCard();
			break;
		case 0x22: // 卸卡测试
			dropCard();
			break;
		case 0x23: //启动测试
			testStatus=1;
			break;
		case 0x24: //关闭测试
			testStatus=0;
			break;
		case 0x25: //卸卡加减速测试
			REF3_H;	
			EN3_H; 
			printf("移动的距离为：%d",params);
			moter3_step(params);
//			EN3_L; 
			break;
			case 0x27: //停止使能
			EN3_L; 
			break;
		case 0x26: //卸卡加减速测试
			addCard();
			break;
		case 0x28: //卸卡电机复位
			REF3_H;	
			EN3_H; 
			moter3_int();
			break;
		case 0x29: //卸卡电机复位
		if(HAL_GPIO_ReadPin(IN4_GPIO_Port,IN4_Pin) == KEY_OFF)
		{
			printf("false");
		}else{
			printf("true");
		}
			break;
		case 0x30: //I2C 读写测试
			Read_equipment_parament();
			break;
		case 0x31: //移动指定卡槽号到测试位置
			testCard(R2);
			break;
		case 0x32: //进行卸卡复位
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
            PC_test();                     //发送检测数据给PC机
            break;
        case 02: //即时 Test
            Immediately_Test(R2,R3,R4);
            break;
        case 03: // 曲线测试
            Graph_test(R2);
            break;
		case 04:// 获取当前卡槽插卡状态:
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

//用来判断当前是否还处于插卡模式
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




//定时器回调函数
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    /* USER CODE BEGIN Callback 0 */
    static uint32_t num = 0;
    /* USER CODE END Callback 0 */
    if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
    /* USER CODE BEGIN Callback 1 */
    else if (htim->Instance == TIM2) // 定时器2基地址
    {
        // 自定义应用程序
        time++;
        num++;
        if (num == 100000) // 每1秒LED灯翻转一次
        {
//            HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0);
//            CAN_senddata(&hcan, TextBuffer);
            num = 0;
        }
    }
	else if(htim->Instance==TIM3){  //控制电机PWM输出
		//电机加减速实现功能。
//		TIM3Fun();
//		printf("motor 加减速测试");
	}	else if(htim->Instance==TIM4){  //控制电机PWM输出
		//电机加减速实现功能。
		TIM4Fun();
//		printf("motor 加减速测试");
	}
	
    /* USER CODE END Callback 1 */
}
