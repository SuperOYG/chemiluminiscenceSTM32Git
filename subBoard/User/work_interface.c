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



//第二次GitHub提交测试
uint16_t github; //test
//测试变量
uint32_t  getNumber;

//中断计时变量
uint32_t time;


uint8_t R0, R1, R2, R3, R4, R5, R6, R7, R8, R9; //用来接收串口下发的指令。

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

//震荡电机移动
int MoveStep;

//Can发送数据函数：
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

// ADC转换值
__IO uint32_t ADC_ConvertedValue;
// 用于保存转换计算后的电压值
float ADC_Vol;

/**
******检测使用的中间变量*********
*/
//当前的温度值：
float CurTemp;
//目标温度：
//float targetTemp=37;
float targetTemp = 39;

//温度上传标志位：
u8 uploadTempFlag;

//PID变量
realizePID rPID;        //pid参数
realizeError rError;    //误差参数
int TMP_PWM = 0;            //控制温度的PWM输出

//定义一个滑轨的移动位置数组：
int slipAddress[15] = {17760, 16358, 0, 0, 0, 0, 0, 0, 0, 0, 8312, 7290, 6385, 5480, 4620};
/*
*  *********************************检测使用的中间变量*******************************
*/
u32 moter3_now;      //Z轴电机当前位置
u32 moter3_next;     //Z轴电机下一步位置

u32 moter1_now;      //X轴电机当前位置
u32 moter1_next;     //X轴电机下一步位置

u32 moter2_now;      //y轴电机当前位置
u32 moter2_next;     //y轴电机下一步位置

u8 motor1Flag;//电机1复位标志位：
u8 motor2Flag;//电机2复位标志位：
u8 motor3Flag;//电机2复位标志位:

//出仓电机复位偏差补偿：
int motor1_ResetParams;
//磁棒电机复位偏差补偿：
int motor2_ResetParams;
//震荡电机复位偏差补偿：
int motor3_ResetParams;

//电机移动速度：
u16 motor1_Speed;
u16 motor2_Speed;
u16 motor3_Speed = 400;

//当前机器状态
uint8_t testStatus;

u8 moter2_RestNum; //震荡电机复位次数。

//下面这3个参数用来做循环测试：
u8  testTimes;//循环测试次数；
u32 moter3_position;//震荡点击移动距离；
u32 heighOfLiquid;//吸液距离；

u8  channel1OfLiquid; //取液孔位
u8  channel2OfLiquid; //吐液孔位

//下面这3个参数是用来做震荡测试：
u8 timesOfShocck;//震荡次数
u32 height1OfShock; //震荡位置
u32 height2OfShock;
//u32 speedOfShock = 200; //震荡速度  单位是 r/min(单位时间电机转的圈数);
u32 speedOfShock = 400; //震荡速度  单位是 r/min(单位时间电机转的圈数);


//吸液起始高度
u16 aspiration_height=7000;
//切换电机模式：
void ChangeGPIOMode(u8 flag, u8 moterId);

//重复吸液函数：
void LoopPickUpTest(u32 LiquidHeight, u32 motor3_Position, u8 times);
//震荡测试函数：
void ShockTest(u32 height1, u32 height2, u32 times);
//延时ms:
void ms_Delay(uint16_t t_ms);


//PID与Error参数初始化函数
void PidAndErrorInit()
{
    rPID.P = 2000;
    rPID.I = 120;
    rPID.D = 600;
    rPID.PWMLIMITE = 9999;    //改参数值状态下,输入14.1V,满功率输出14V;
}

//启动加热
void Start_TempCon(void)
{
    HAL_TIM_Base_Start_IT(&htim2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
}
//停止加热
void Stop_TempCon(void)
{
    HAL_TIM_Base_Stop_IT(&htim2);
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
//    Hot_OFF;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
}



//电机1 复位：

void moter1_int2(u16 CurPosition, u16 step)
{
    u16 moveStep = CurPosition + 400;
    DIR1_L;
    Moter1_Run(1, 1, moveStep); //10000：为运动最远处的步数，待定。
    DIR1_H;

    moter1_now = 0;
    HAL_Delay(200);
    Moter1_Run(0, 1, step); //碰到开关，回拨一小步。
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
        Stepper_Move_S(60, 200, 0.01f, -400); //回拨一小步。
        while (Stepper.status != STOP)
        {
            HAL_Delay(10);
        }
        moter3_now = 0;
    }
    params = params + motor1_ResetParams; //将出仓补偿数添加到移动步数。
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
        Stepper_Move_S(60, 200, 0.01f, -400); //震荡电机再次往上移动200步，避免下面滑动碰撞。
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
    params = params + motor2_ResetParams; //将磁棒补偿数添加到移动步数。
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
    params = params + motor3_ResetParams; //将震荡补偿数添加到移动步数。
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
        Stepper_Move_S(60, 200, 0.01f, -400); //回拨一小步。
        while (Stepper.status != STOP)
        {
            HAL_Delay(10);
        }
        moter3_now = 0;
    }
}
void init_PA0();
/**********************************************************
 * @brief 单元组的电机3轴复位
 * @author  hang
 * @date  
 **********************************************************/
void init()
{
    HAL_Delay(2000);
    moter2_int(200);//磁棒复位
    if (HAL_GPIO_ReadPin(IN3_GPIO_Port, IN3_Pin) != 1) //判断一下震荡电机，当前所处的位置：
    {
        motor3Flag = 1;
        Stepper_Move_S(60, 200, 0.01f, -10000);
        while (Stepper.status != STOP)
        {
            HAL_Delay(10);
        }
        motor3Flag = 0;         //触发开关后，在往上退400步。 避免移动磁棒时，碰到刺破枪头。
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
        motor3Flag = 0;         //触发开关后，在往上退400步。 避免移动磁棒时，碰到刺破枪头。
        Stepper_Move_S(60, 200, 0.01f, -400);
        while (Stepper.status != STOP)
        {
            HAL_Delay(10);
        }
    }
    moter3_now = 0; //这是把触碰到开关后的向上再移动400的位置，作为运动坐标原点。
    moter1_int(200);
//    PidAndErrorInit();
//    Start_TempCon();  //开启温控
	
	//初始化滑道数组：
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
	
	init_PA0();//初始化阀门开关。
	
}

/**
*   @brief:  上传实时温度
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
    *   @brief: 换轨道吸液测试：
    *   @param:  height: 吸液高度；channel1:取液槽;channel：吐液槽。
    *   @return: void
    *   @author Hang
    *   @date: 2023-05-24
    */
void aspirationTest(u16 height, u8 channel1, u8 channel2)
{
    u16 moter1_next;  //出仓
    u16 moter2_next; //磁棒距离
	//1.磁棒到达吸液起始点
    moter2_move(aspiration_height); //移到吸液起始位置，移动到7000步，开始实验。;2023-09-11  调整为6500 。测试是否能吐干净。
    //2.出仓电机：移到取液位：
    moter1_next = slipAddress[channel1 - 1];
    moter1_move(moter1_next);
    //3.震荡电机：下移指定距离：
    if (channel1 == 15) //当吸取样本槽
    {
//		moter3_move(2600);
//		moter3_move(2390); //0918
		moter3_move(2450);
    }
    else      //非样本槽：
    {
//		moter3_move(5400);
		moter3_move(5400);//0918
    }
    //4.磁棒上移：吸液
    moter2_next = aspiration_height - height;
    moter2_move(moter2_next);
    //5.震荡电机：复位
    moter3_reset();
//    //6.磁棒上移：
//    moter2_next = moter2_next;
//    moter2_move(moter2_next);
    //7.出仓电机：吐液位
    moter1_next = slipAddress[channel2 - 1];
    moter1_move(moter1_next);
   //8.磁棒下移：移动8000位置，吐液。
	if(channel2==1){ //反应仓吐液
//		moter3_move(4300);
		moter3_move(4250);
		moter2_move(9200);
		aspiration_height=9200;
		
//		moter2_move(10500);
//		aspiration_height=10500;
	}
	else
	{  //移动9000位置，吐液。
		moter3_move(4900);
		moter2_move(10500);
		aspiration_height=10500;
//		moter2_move(9000);
//		aspiration_height=9000;
	}
    //9.震荡电机：下降到吐液位置
	    moter3_reset();//震荡电机复位.
}

/**
*   @brief: 震荡测试：
*   @param:  height1: 是震荡电机下降最低位置，height2:是震荡电机震荡高度。
             times：震荡次数。
*   @return: void
*   @author Hang
*   @date: 2023-05-20
*/
void ShockTest(u32 height1, u32 height2, u32 times)
{	    
	
		aspiration_height=7000; //将吸液起始位置归零复位。
		motor3Flag = 0;
		height1=height1+motor3_ResetParams;  //添加震荡电机补偿参数。
		Stepper_Move_S(60, 200, 0.01f, height1);
		moter3_now = height1;
		while (Stepper.status != STOP)
			while (Stepper.status != STOP)
			{
				HAL_Delay(10);
			}
			
		moter2_int(200);  //磁棒复位	
		for (int i = 0; i < 75; i++)  //改为75次。
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
		//震荡完成磁棒要复位吗
		HAL_Delay(times * 1000); //震荡完成之后的反应等待时间。
}

//枪头特殊移动：
void gunHeadMove(u8 time, u16 step)
{
    u32 MoveStep;
    ChangeGPIOMode(2, 3); //切换引脚配置
	step=step+motor3_ResetParams; //添加移动补偿
    if (step > moter3_now)
    {
        MOTOR_DIR(CCW);//下
        MoveStep = step - moter3_now;
    }
    else if (step < moter3_now)
    {
        MOTOR_DIR(CW);//上
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
		ChangeGPIOMode(1, 3); //切回加减速模式。
}

/**
震荡电机变速移动：
*/



//电机3的引脚变速移动 flag：类型，移动距离。
void moter3Move3(u8 flag, u32 params)
{
    if (flag == 0x01) //取枪头
    {
        moter3_move(4500);
		gunHeadMove(4, params);
//        moter3_move(0);
		moter3_reset();
    }
    else if (flag == 0x02) //卸载磁棒套后，震荡电机向上运动。 //
    {
		gunHeadMove(4, 4000);
//		moter3_move(params);
			moter3_reset();
    }
    else if (flag == 0x03) //吸附动作
    {		  			
		moter3_move(4500);
		gunHeadMove(15, 5100); 
		HAL_Delay(6000); //第一个位置等待延时。
		gunHeadMove(15, params);
		HAL_Delay(8000); //第二个位置等待延时。	
		gunHeadMove(10, 4500);//
//		moter3_move(0);	
		moter3_reset();
    }
	else if(flag == 0x04) //吸附动作2 在清洗液中进行 
	{
		moter3_move(3000);
		gunHeadMove(15, 4600); 
		HAL_Delay(8000); //第一个位置等待延时。
		gunHeadMove(15, params);
		HAL_Delay(8000); //第二个位置等待延时。	
		gunHeadMove(10, 3000);//
//		moter3_move(0);
		moter3_reset();		
	}
	else if(flag==0x05){
		gunHeadMove(10, params);
	}
}

//对收到来自CAN和串口的指令进行分析处理




/**********************************************************
 * @brief   对Test获取到的指令数据，进行指令解析
 * @param   
 * @author  hang
 * @date  
 **********************************************************/

void  getPraVue()
{
    uint32_t params;
    params = (R3 - 0x30) * 10000 + (R4 - 0x30) * 1000 + (R5 - 0x30) * 100 + (R6 - 0x30) * 10 + (R7 - 0x30);

    int moveStep;  //震荡电机移动距离
    switch (R0)
    {
    case 0x01:                 //01：表示和电机运动有关
        switch (R1)
        {
        case 0x00://电机复位
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
						
						
						
            else if (R2 == 0x04) //滑台   //设置电机复位参数 正补偿
            {
                motor1_ResetParams = params;
            }
            else if (R2 == 0x05) //磁棒
            {
                motor2_ResetParams = params;
            }
            else if (R2 == 0x06) //震荡
            {
                motor3_ResetParams = params;

            }
						
						
            else if (R2 == 0x07) //滑台
            {
                motor1_ResetParams = -params;
            }
            else if (R2 == 0x08) //磁棒
            {
                motor2_ResetParams = -params;
            }
            else if (R2 == 0x09) //震荡
            {
                motor3_ResetParams = -params;
            }
            break;
        case 0x01://电机移动          
            if (R2 == 0x01)
            {
                moter1_move(params);
            }
            else if (R2 == 0x02)
            {
                moter2_move(params);
            }
            else if (R2 == 0x03) //使用S弯加减速：
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
            else if (R2 == 0x04) //推杆电机速度
            {
                motor1_Speed = params;
            }
            else if (R2 == 0x05) //修改磁棒速度
            {
                motor2_Speed = params;
            }
            else if (R2 == 0x06) //修改震荡电机速度
            {
                motor3_Speed = params;
            }
            break;
        case 0x02:

            break;
        case 0x03:

            break;
        case 0x04: //吸液单元测试：          这个就是这次要实现的功能函数。2025年3月23日21:12:03
            if (R2 == 0x01) //吸液高度
            {
                heighOfLiquid = params;
            }
            else if (R2 == 0x02) //设置震荡
            {
                moter3_position = params;
            }
            else if (R2 == 0x03) //设置次数
            {
                testTimes = params;
                LoopPickUpTest(heighOfLiquid, moter3_position, testTimes);
            }
            else if (R2 == 0x04) //设置吸液孔位
            {
                channel1OfLiquid = params;
            }
            else if (R2 == 0x05) //设置吐液孔位
            {
                channel2OfLiquid = params;
            }
            else if (R2 == 0x06) //设置吸液量
            {
                heighOfLiquid = params;
                aspirationTest(heighOfLiquid, channel1OfLiquid, channel2OfLiquid);
            }
            break;
        case 0x05: //电机震荡测试：
            if (R2 == 0x02) //设置震荡电机速度
            {
                speedOfShock = params;
            }
            else if (R2 == 0x03)//震荡位置
            {
                height1OfShock = params;
            }
            else if (R2 == 0x04) //振幅高度
            {
                height2OfShock = params;
            }
            else if (R2 == 0x05) //震荡次数
            {
                timesOfShocck = params;
                ShockTest(height1OfShock, height2OfShock, timesOfShocck);
            }
            break;
        default:
            break;
        }
        break;
    case 0x02:         //表示和温度控制有关
        switch (R1)
        {
        case 01:
            if (R2 == 0x01) //温控开启
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
            if (R2 == 0x01) //上传温度
            {
                uploadTempFlag = 1;
            }
            else if (R2 == 0x02)
            {
                uploadTempFlag = 2;
            }
            break;
        case 03: //设定目标温度
            targetTemp = (R6 - 0x30) * 10 + (R7 - 0x30) + 0.5; //补偿个0.5°。
            if (targetTemp > 50)
            {
                targetTemp = 25;
            }
            break;
        case 04: //设置温控PID参数：s
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
    case 0x03:         // 获取一些开关量信息：
        switch (R1)
        {
        case 01:
            if (R2 == 0x01) //出仓电机光耦检测
            {
                if (HAL_GPIO_ReadPin(IN1_GPIO_Port, IN1_Pin) == 0)
                {
                }
                else
                {


                }
            }
            else if (R2 == 0x02) //吸液电机光耦检测
            {

                if (HAL_GPIO_ReadPin(IN2_GPIO_Port, IN2_Pin) == 0)
                {


                }
                else
                {


                }
            }
            else if (R3 == 0x03) //震荡电机光耦检测
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
    case 0x04: //电机参数补偿
        params = (R4 - 0x30) * 1000 + (R5 - 0x30) * 100 + (R6 - 0x30) * 10 + (R7 - 0x30);
        if (R2 == 0x01) //设置出仓电机
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
        else if (R2 == 0x02) //设置磁棒电机
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
        else if (R2 == 0x03) //设置震荡电机
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
    case 0x05: //电机切换IO移动测试
        gunHeadMove(R1, params);
        break;
    case 0x06:
        if (R1 == 0x01) //刺破
        {



        }
        else if (R1 == 0x02) //吸液
        {



        }
        else if (R1 == 0x03) //震荡
        {


        }
        else    //默认状态
        {

        }
        break;
    case 0x07:
        moter3Move3(R1, params); //震荡电机变速移动测试。
        break;
    default:
        break;
    }
}



/**********************************************************
 * @brief   一次完整的TIP  Z轴吸液前复位-》 Z轴下移吸液-》复位悬停10S-》吐液；
 * @param   
 * @author  hang
 * @date  
 **********************************************************/
void SingleTest(u32 params, u32 motor3_position)
{
    params = 9000 - params;
    int moveStep;
    //限制下磁棒移动距离：
    if (params < 5000)
    {
        params = 5000;
    }
    else if (params > 9500)
    {
        params = 5000;
    }
		
		
    //1.磁棒电机向上复位
    moter2_int(200);
    //2. 震荡电机复位电机向上复位
    motor3Flag = 1;
    Stepper_Move_S(60, 200, 0.01f, -10000);
    while (Stepper.status != STOP);
    HAL_Delay(10);
    moter3_now = 0;
    //3.磁棒下降下降10000;
    moter2_step(10000, 0);
    //3.磁棒上升1000步：吸少量空气，方便后期吐液
    moter2_step(9000, 0);
    //4.震荡电机下降6000； 移液泵下探到试剂液面
    motor3Flag = 0;
    moveStep = motor3_position - moter3_now;
    Stepper_Move_S(60, 200, 0.01f, moveStep);
    moter3_now = 5300;
    HAL_Delay(2000);
    //5.磁棒吸液
    moter2_step(params, 0);
    HAL_Delay(500);
    //6.震荡电机复位电机向上复位
    motor3Flag = 1;
    Stepper_Move_S(60, 200, 0.01f, -10000);
    while (Stepper.status != STOP);
    HAL_Delay(10);
    moter3_now = 0;
    HAL_Delay(500);
//    //7.磁棒电机上升50；吸50步空气,防止液体滴落.             干脆先不吸空气
//    moter2_step(params - 50, 0);
    HAL_Delay(10000);
    //8.磁棒电机向下移动到10000，进行吐液。
    moter2_step(10500, 0);
    HAL_Delay(3000);
}

/**********************************************************
 * @brief   循环吸液测试
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
 * @brief   主任务函数，解析CAN口中断接收到的数据
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
            //1.拉高CAN总线的指示GPIO的电平；
            HAL_GPIO_WritePin(CAN_Signal_GPIO_Port, CAN_Signal_Pin, GPIO_PIN_SET);
            getPraVue();
            //2.拉低CAN总线的指示GPIO的电平；
            HAL_Delay(100);
            CAN_senddata(&hcan, TextBuffer, 0x11);
            HAL_GPIO_WritePin(CAN_Signal_GPIO_Port, CAN_Signal_Pin, GPIO_PIN_RESET);
//          HAL_Delay(100);
        }
        Rx_end = 0;
    }
}


/**********************************************************
 * @brief   //延时1s函数；
 * @author  hang
 * @date  
 **********************************************************/
void ms_Delay(uint16_t t_ms)
{
    uint32_t t = t_ms * 3127;
    while (t--);
}


/**********************************************************
 * @brief   对采集的ADC值求平均
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
 * @brief   获取温度值。
 * @author  hang
 * @date  
 **********************************************************/
void getTemp()
{
    ADC_ConvertedValue = Get_Adc_Average(6);
    ADC_Vol = (float) ADC_ConvertedValue / 4096 * 3.3; // 读取转换的AD值
    float r = (ADC_Vol * 10000.0) / (3.3 - ADC_Vol);
    double result = pow(r, -0.1588);
    CurTemp = 609.4 * result - 116.1;
}




/**********************************************************
 * @brief   //定时更改TIM_CH1 的输出PWM值,即控制加热丝的工作。
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
    rError.Current_Error = targetTemp - CurTemp;  //求得当前误差
    TMP_PWM = (int)PID_Realize(&rError, &rPID);   //根据PID参数求得输出的PWM
    if (TMP_PWM < 0)
    {
        TMP_PWM = 0;
    }
//    TMP_PWM = rPID.PWMLIMITE - TMP_PWM; //占空比越小，场效应管导通比例越大，此处做反向；    这块好像不用做反向了，在精子检测上不需要反向。
    __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, TMP_PWM);
}



/**********************************************************
 * @brief   系统定时器回调函数，用来触发电机状态运行
 * @author  hang
 * @date  
 **********************************************************/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    /* USER CODE BEGIN Callback 0 */
    static uint32_t num = 0;
    static uint32_t num1 = 0;
    /* USER CODE END Callback 0 */
    if (htim->Instance == TIM2) // 定时器2基地址
    {
        // 自定义应用程序
        time++;
        num++;
////        //定时触发TIM2 定时器函数：1s触发一次定时器判断。
        if (num == 100)
        {
            timer2();
            num = 0;
        }
    }
    else if (htim->Instance == TIM4)
    {
        // 自定义应用程序
        num1++;
        if (num1 == 1000) // 每1秒LED灯翻转一次
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
 * @brief   //切换GPIO 输出模式:用来切换PWM和普通IO模式。
 * @author  hang
* @date  2025年3月17日14:46:47 
 **********************************************************/

void ChangeGPIOMode(u8 flag, u8 moterId)
{
    if (flag == 1) //切换为PWM模式
    {
        if (moterId == 1)
        {
//               MX_TIM4_Init();
        }
        else if (moterId == 2)
        {
            MX_TIM3_Init();
        }
        else if (moterId == 3)  //震荡电机
        {
            MX_TIM1_Init();
        }
    }
    else if (flag == 2) //切换为普通IO模式
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
            //将震荡电机切换成 普通IO口运行。
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
 * @brief   将温控的PWM输出引脚重新设置为 普通IO 输出模式，控制MOS通断
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






