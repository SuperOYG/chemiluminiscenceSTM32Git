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
extern MessageQueue msgQueue;  // 全局消息队列

//中断计时变量
uint32_t time;
uint8_t R0, R1, R2, R3, R4, R5, R6, R7, R8, R9; //用来接收串口下发的指令。

//当前使用的串口位；
uint8_t CurUartNum;

uint8_t  RecieveBuffer[1];//暂存接收到的字符
uint8_t  Rx_end;  //指令接收完成标志位
uint8_t  RxLine;  //rxbuf接收的数据长度为:RxLine+1;
uint8_t  rxbuf[50];//收到的数据存放处
uint8_t TextBuffer[8] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x44, 0x55, 0x55};

//震荡电机移动
int MoveStep;

/**
******检测使用的中间变量*********
*/
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
int motor1_ResetParams=200; //2025年3月24日20:50:37
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
u16 aspiration_height=8000;
//切换电机模式：
void ChangeGPIOMode(u8 flag, u8 moterId);

//重复吸液函数：
void LoopPickUpTest(u32 LiquidHeight, u32 motor3_Position, u8 times);
//震荡测试函数：
void ShockTest(u32 height1, u32 height2, u32 times);
//延时ms:
void ms_Delay(uint16_t t_ms);



//电机细分数：崔凯电路板的电机细分是32；我的电机1,2是4细分； 震动电机的细分是8细分；
//所以，我要在我原有程序的基础上，加一个运动倍数的系数，电机1,2 是8 ，电机3是4细分；
//uint8_t Motor1_2Factor=8;
//uint8_t Motor3Factor=4;


//电机1 复位：
void moter1_int2(u16 CurPosition, u16 step)
{
    u16 moveStep = CurPosition + 400;
    DIR1_L;
    Moter1_Run(1, 0, moveStep); //10000：为运动最远处的步数，待定。
    DIR1_H;

    moter1_now = 0;
    HAL_Delay(200);
    Moter1_Run(0, 0, step); //碰到开关，回拨一小步。
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

/**********************************************************
 * @brief 单元组的电机3轴复位
 * @author  hang
 * @date  
 **********************************************************/
void init()
{
    HAL_Delay(2000);
    moter2_int(200);//磁棒复位
    if (HAL_GPIO_ReadPin(IN5_GPIO_Port, IN5_Pin) != 1) //判断一下震荡电机，当前所处的位置：
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

	
	//初始化滑道数组：
					int n = (slipAddress[1] - slipAddress[10]) / 9;
					for (int i = 2; i < 10; i++)
					{
							slipAddress[i] = slipAddress[1] - n * (i - 1);
					}					
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
		aspiration_height=8000;//吸液磁棒初始位置 
	//1.磁棒到达吸液起始点
    moter2_move(aspiration_height); //移到吸液起始位置，移动到7000步，开始实验。;2023-09-11  调整为6500 。测试是否能吐干净。
    //2.出仓电机：移到取液位：
    moter1_next = slipAddress[channel1 - 1];
    moter1_move(moter1_next);
    //3.震荡电机：下移指定距离：
    if (channel1 == 15) //当吸取样本槽
    {
		moter3_move(2450);
    }
    else      //非样本槽：
    {
//		moter3_move(5400);//0918
		moter3_move(4400);//0918
    }
    //4.磁棒上移：吸液
    moter2_next = aspiration_height - height;
    moter2_move(moter2_next);
    //5.震荡电机：复位
    moter3_reset();
//    //6.磁棒上移：吸空气
    moter2_next = moter2_next-200;
    moter2_move(moter2_next);
		moter1_reset(0);
}

/**********************************************************
 * @brief   悬停吐液
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
//    u16 moter1_next;  //出仓
//    u16 moter2_next; //磁棒距离
//		aspiration_height=8000;//吸液磁棒初始位置 
//	//1.磁棒到达吸液起始点
//    moter2_move(aspiration_height); //移到吸液起始位置，移动到7000步，开始实验。;2023-09-11  调整为6500 。测试是否能吐干净。
//    //2.出仓电机：移到取液位：
//    moter1_next = slipAddress[channel1 - 1];
//    moter1_move(moter1_next);
//    //3.震荡电机：下移指定距离：
//    if (channel1 == 15) //当吸取样本槽
//    {
//		moter3_move(2450);
//    }
//    else      //非样本槽：
//    {
////		moter3_move(5400);//0918
//		moter3_move(4400);//0918
//    }
//    //4.磁棒上移：吸液
//    moter2_next = aspiration_height - height;
//    moter2_move(moter2_next);
//    //5.震荡电机：复位
//    moter3_reset();
////    //6.磁棒上移：吸空气
//    moter2_next = moter2_next-200;
//    moter2_move(moter2_next);
//    //7.出仓电机：吐液位
//    moter1_next = slipAddress[channel2 - 1];
//    moter1_move(moter1_next);
//   //8.磁棒下移：移动8000位置，吐液。
//	if(channel2==1){ //反应仓吐液
//		moter3_move(4250);
//		moter2_move(8500);
//		aspiration_height=8500;
//	}
//	else
//	{  //移动9000位置，吐液。
//		moter3_move(4900);
//		moter2_move(8500);
//		aspiration_height=8500;
//	}
//    //9.震荡电机：下降到吐液位置
//	    moter3_reset();//震荡电机复位.
//}






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
            }else if(R2==0x07){
							aspirationTest_drop();
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
    case 0x02:         //控制气阀
        switch (R1)
        {
        case 01: //左侧阀门
            if (R2 == 0x01) //
          {
								HAL_GPIO_WritePin(PUMP1_GPIO_Port,PUMP1_Pin,GPIO_PIN_SET);
						}
            else if (R2 == 0x02)
            {
								HAL_GPIO_WritePin(PUMP1_GPIO_Port,PUMP1_Pin,GPIO_PIN_RESET);
            }
            break;
        case 02: //右侧阀门
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

void sysInit(void){

		Queue_Init(&msgQueue);
    // 首次启动接收（从 currentFrame.data[0] 开始）
    HAL_UART_Receive_IT(&huart1, &currentFrame.data[0], 1);
		DIR2_L;
		HAL_Delay(2000);
		EN2_H;
		EN1_H;	
		EN3_H;	
	  init();	
}




/**********************************************************
 * @brief   主任务函数，解析CAN口中断接收到的数据
 * @author  hang
 * @date  
 **********************************************************/
void test()
{
	
		 Frame frame;
        if (Queue_Pop(&msgQueue, &frame)) {
            // 解析数据帧（示例：打印帧内容）
            for (uint8_t i = 0; i < frame.length; i++) {
                printf("%02X ", frame.data[i]);
            }
            printf("\n");

            // 在此添加你的具体指令解析逻辑
            // 例如：if (frame.data[2] == 0x01) { ... }
						
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

}

/**********************************************************
 * @brief   获取温度值。
 * @author  hang
 * @date  
 **********************************************************/
void getTemp()
{

}




/**********************************************************
 * @brief   //定时更改TIM_CH1 的输出PWM值,即控制加热丝的工作。
 * @author  hang
 * @date  
 **********************************************************/
void timer2()
{

	
	
	
}



/**********************************************************
 * @brief   系统定时器回调函数，用来触发电机状态运行
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
// * @brief   //切换GPIO 输出模式:用来切换PWM和普通IO模式。
// * @author  hang
//* @date  2025年3月17日14:46:47 
// **********************************************************/

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








