#include "motor.h"
#include "main.h"
#include "eeprom.h"
#include "Stdio.h"
#include "S_Moter1.h"
#include "S_Moter3.h"

extern volatile uint32_t time;

uint32_t x_now; //转盘电机移动位移
uint32_t y_now; //测试电机移动位移
uint32_t z_now; //退卡电机移动位移


uint8_t  Channel_now; //当前镜头所处在的测试卡位;

u8 MotorX_speed = 60;  //横移方向电机速度
u8 MotorY_speed = 15;  //测试方向电机速度
u8 MotorZ_speed = 20;  //卸卡方向电机速度

u8 speedgear=4;//速度档位 
extern u8 NumberOfCards;

extern u8 testStatus;

//当前测试的试剂的卡槽序号
extern u8 testIndex;


extern u8 StartStep;

extern u8 SingleStep;


extern u32 moter3_now;      //Z轴电机当前位置
extern u32 moter3_next;     //Z轴电机当前位置

extern u8 motor3Flag; //  Z轴电机复位标志位

extern u32 moter1_now;      //x轴电机当前位置
extern u32 moter1_next;     //x轴电机当前位置

extern u8 motor1Flag; //  x轴电机复位标志位

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
*   @brief: 检测方向电机复位  电路板左数第1个电机
*   @param:  delay：速度，direction ：为0 向外复位， 为1向内侧复位。 
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
//	GENERAL_TIM_Init();							/*定时器初始化*/
//	TIM_Cmd(GENERAL_TIM, DISABLE);				//关闭定时器
//	TIM_Cmd(GENERAL_TIM, ENABLE);				//开启定时器	
	time = 0;
//	while(HAL_GPIO_ReadPin(IN1_GPIO_Port,IN1_Pin) == KEY_ON)   //PC7     P7接口
	while(HAL_GPIO_ReadPin(IN5_GPIO_Port,IN5_Pin) == KEY_ON)   //    P13接口
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
		y_now = 0;  //位置坐标归零
		EN1_L;
	}
	else {
	EN1_H;
	DIR1_L;
//	GENERAL_TIM_Init();							/*定时器初始化*/
//	TIM_Cmd(GENERAL_TIM, DISABLE);				//关闭定时器
//	TIM_Cmd(GENERAL_TIM, ENABLE);				//开启定时器	
	time = 0;
	while(HAL_GPIO_ReadPin(IN1_GPIO_Port,IN1_Pin) == KEY_ON)   //PC7     P7接口
//	while(HAL_GPIO_ReadPin(IN5_GPIO_Port,IN5_Pin) == KEY_ON)   //    P13接口
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
		y_now = 0;  //位置坐标归零
		EN1_L;
	}
}



/**
*   @brief: 转盘复位  左数第二个电机
*   @param:  void
*   @return: void
*   @author Hang
*   @date:  2023-03-30
*/
void Ka2_InitBack(u8 delay)
{	
	u16 step1 = 1334; //经过限位开关后继续回退距离。
	u32 stop_flag = 0;
	Delay_moter(1000);
	EN2_H;
	DIR2_H;
//	GENERAL_TIM_Init();						/*定时器初始化*/
//	TIM_Cmd(GENERAL_TIM, DISABLE);				//关闭定时器
//	TIM_Cmd(GENERAL_TIM, ENABLE);				//开启定时器	
	time = 0;
	while(HAL_GPIO_ReadPin(IN6_GPIO_Port,IN6_Pin) == KEY_OFF)    // 光耦5  ：P8接口
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
*   @brief: 转盘复位后移动距离  左数第二个电机
*   @param:  void
*   @return: void
*   @author Hang
*   @date:  2023-03-30
*/
void Ka2_InitBack2(u8 delay,u16 distance)
{	
	u16 step1 = distance; //经过限位开关后继续移动距离。
	u32 stop_flag = 0;
	Delay_moter(1000);
	EN2_H;
	DIR2_H;
	u32 totalStep;
//	GENERAL_TIM_Init();						/*定时器初始化*/
//	TIM_Cmd(GENERAL_TIM, DISABLE);				//关闭定时器
//	TIM_Cmd(GENERAL_TIM, ENABLE);				//开启定时器	
	time = 0;
	while(HAL_GPIO_ReadPin(IN6_GPIO_Port,IN6_Pin) == KEY_OFF)    // 光耦5  ：P8接口
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
*   @brief: 退卡电机复位  左数第3个电机
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
//	GENERAL_TIM_Init();							/*定时器初始化*/
//	TIM_Cmd(GENERAL_TIM, DISABLE);				//关闭定时器
//	TIM_Cmd(GENERAL_TIM, ENABLE);				//开启定时器	
	time = 0;
	while(HAL_GPIO_ReadPin(IN4_GPIO_Port,IN4_Pin) == KEY_ON)          //PB13    P12接口 
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
*   @brief: 电机初始化，电机位置归零
*   @param:  void
*   @return: void
*   @author Hang
*   @date: 
*/
void Motor_init()
{
  //M1_ON;M2_ON;M3_OFF;  //110  16细分
	REF1_H;
	REF2_H;
	REF3_H;
    EN1_H;
	EN2_H;
	EN3_H;
	moter3_int();//卸卡复位（带加减速） 
//	Ka3_InitBack(1);   // 退卡电机复位
//	HAL_Delay(1000);
	Ka1_InitBack(1,1);   // 测试电机复位
//	HAL_Delay(500);
//	Ka2_InitBack(1);   //  转盘复位
	Ka2_InitBack2(1,1280);
}







/**
//------------------CurUser--------------
*   @brief: 测试方向的电机移动控制  以外侧零点为测试原点。
*   @param:  delay:脉冲周期
			 distance:位移距离
*   @return: void
*   @author Hang
*   @date: 2023-03-30
*/
void Ka1Moter_Step(u8 delay,u32 distance)
{  
	u16 step;
	EN1_H;
//	if(distance>10000) distance=10000;
	printf("y轴移动距离%d",distance);
	u32 Ka_next = distance;
	if(Ka_next>y_now){step = Ka_next-y_now;DIR1_H;}    //
	else if(Ka_next<y_now) {step = y_now - Ka_next;DIR1_L;}
	else step = 0;
//	GENERAL_TIM_Init();			/*定时器初始化*/
//	TIM_Cmd(GENERAL_TIM, DISABLE);	//关闭定时器
//	TIM_Cmd(GENERAL_TIM, ENABLE);		//开启定时器
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
*   @brief: 转盘位移控制
*   @param:  delay:脉冲周期
			 distance:位移距离
*   @return: void
*   @author Hang
*   @date: 2023-3-30
*/
void Ka2Moter_Step(u8 delay,u32 distance)
{                          
	u16 step;
//	if(distance>14000) distance=14000;
	u32 Ka_next = distance;
//	if(y_now!=0){      // 横移之前一定判断测试方向是否归零
////		Ka3_InitBack(1);   // 退卡电机复位
////		HAL_Delay(200);
////		Ka1_InitBack(1,1);   // 测试电机复位
////		HAL_Delay(200);
//	} 
	EN2_H;
	if(Ka_next>x_now){step = Ka_next-x_now;DIR2_H;}
	else if(Ka_next<x_now) {step = x_now - Ka_next;DIR2_L;}  //正常情况不允许回转。只能顺转，这里只做测试用。
	else step = 0;
//	GENERAL_TIM_Init();			/*定时器初始化*/
//	TIM_Cmd(GENERAL_TIM, DISABLE);	//关闭定时器
//	TIM_Cmd(GENERAL_TIM, ENABLE);		//开启定时器
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
	printf("转盘当前位置为%d",x_now);
//	EN2_L;
}
/**
*   @brief: 卸卡电机移动
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
	printf("Z轴移动距离%d",distance);
	u32 Ka_next = distance;
	if(Ka_next>z_now){step = Ka_next-z_now;DIR3_H;}    //
	else if(Ka_next<z_now) {step = z_now - Ka_next;DIR3_L;}
	else step = 0;
//	GENERAL_TIM_Init();			/*定时器初始化*/
//	TIM_Cmd(GENERAL_TIM, DISABLE);	//关闭定时器
//	TIM_Cmd(GENERAL_TIM, ENABLE);		//开启定时器
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
*   @brief: 加卡动作
*   @param:  void
*   @return: void
*   @author Hang
*   @date: 2023-3-30
*/

void addCard(){
	printf("进入插卡等待模式");
	while(NumberOfCards<20&&testStatus==0){
		HAL_Delay(2000);
		if(HAL_GPIO_ReadPin(IN2__GPIO_Port,IN2__Pin) == KEY_OFF
			&&HAL_GPIO_ReadPin(IN1__GPIO_Port,IN1__Pin) == KEY_OFF){
			NumberOfCards++;
//			Ka2_InitBack2(1,480*NumberOfCards+1280);
			Ka2Moter_Step(1,480*NumberOfCards); //转盘转动一格
			printf("当前有卡插入");
		}else{
			printf("暂时无卡插入");
		}
		printf("testStatus=%d",testStatus);
	}
}

//u8 checkCard(){

//	if(){
//	}




//}
/**
*   @brief: 卸卡动作，从测试扫描完成点处将试剂卡转移到卸载口卸载。
*   @param:  void
*   @return: void
*   @author Hang
*   @date: 2023-3-30
*/
void dropCard(){
	Ka1Moter_Step(1,0);   //测试电机向外侧复位
	HAL_Delay(500);
	Ka2Moter_Step(1,2400); // 转盘移动
	HAL_Delay(500);
	moter3_step(35000);// 卸卡移动 
	HAL_Delay(500);
	moter3_int();//卸卡复位
	HAL_Delay(500);
	Ka2_InitBack2(1,1280);// 转盘复位
	NumberOfCards--;
}


/**
*   @brief: 测卡上卡动作
*   @param:  testId : 上位机指定当前测试的卡槽序号
*   @return: void
*   @author Hang
*   @date: 2023-3-30
*/

void testCard(u8 testId){
	testIndex=testId;
		if(NumberOfCards!=0){
	// 将下一张试剂卡槽移动到检测位
	u8 moveStep=(testIndex-1)*equipment_parament_1+equipment_parament_2;		
	Ka2Moter_Step(1,moveStep);   	
	}  
	Ka1Moter_Step(1,equipment_parament_4);   //移动到测试位置。
}



void Delay_moter3(u16 n)
{
	unsigned int i;
	for(;n>0;n--){for(i=100;i>0;i--);}
}

/***************   带加减速控制电机3***************/
void moter3_1step(u8 speed)      
{  
    Delay_moter3(speed);
	CP3_TOGGLE;
}

void moter3_step(u32 distance)//distance = 坐标;
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
 printf("step 的大小为：%d",step);
  if(step > 500)							
		{Moter3_Run(1,0,step);}
	if(step == 500)							
		{Moter3_Run(1,1,step);}
	if((step > 0)&&(step < 500))
		{Moter3_Run(1,2,step);}
	if(step == 0)								
		{;}
  moter3_now = moter3_next; 
  //1.将当前Z轴电机写入eeprom;
  I2C_write2Byte(equipment_addres_7, moter3_now);				
}


//电机3 复位：
void moter3_int()
{   	
	motor3Flag=1;
	u32 step;
      DIR3_L;
 step=I2C_get2Byte(equipment_addres_7);
 printf("moter3_now 的大小为：%d",moter3_now);	
 printf("step 的大小为：%d",step);
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



/***************   带加减速控制电机1**************/
void moter1_1step(u8 speed)      
{  
    Delay_moter3(speed);
	CP1_TOGGLE;
}

void moter1_step(u32 distance)//distance = 坐标;
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
 printf("step 的大小为：%d",step);
  if(step > 500)							
		{Moter1_Run(1,0,step);}
	if(step == 500)							
		{Moter1_Run(1,1,step);}
	if((step > 0)&&(step < 500))
		{Moter1_Run(1,2,step);}
	if(step == 0)								
		{;}
  moter1_now = moter1_next; 
//  //1.将当前Z轴电机写入eeprom;
//  I2C_write2Byte(equipment_addres_7, moter1_now);				
}


//电机1 复位：
void moter1_int()
{   	
	motor3Flag=1;
	u32 step;
      DIR3_L;
 step=I2C_get2Byte(equipment_addres_7);
 printf("moter1_now 的大小为：%d",moter1_now);	
 printf("step 的大小为：%d",step);
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








////扫描使用的Y轴移动函数：
//void Ka1Moter_RunStep(u8 delay,u32 distance)
//{  
//	u16 step;
////	EN1_H;
////	if(distance>11000) distance=11000;
//	u32 Ka_next = distance;
//	if(Ka_next>y_now){step = Ka_next-y_now;DIR1_L;}
//	else if(Ka_next<y_now) {step = y_now - Ka_next;DIR1_H;}
//	else step = 0;
////	GENERAL_TIM_Init();			/*定时器初始化*/
////	TIM_Cmd(GENERAL_TIM, DISABLE);	//关闭定时器
////	TIM_Cmd(GENERAL_TIM, ENABLE);		//开启定时器
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
//*   @brief：电压扫描测试完成后，电机卸卡以及归零操作
//*   @param:  
//*   @return: 
//*   @author Hang
//*   @date: 2023-01-15
//*/
//void Moter_DropCard(){
////1.测试方向向后复位
//	Ka1_InitBack(MotorY_speed);
////2.横移方向向左偏移卸卡位置
//	Ka2Moter_Step(1,x_now-equipment_parament_3);
////3.测试方向外推至最外端。
//	Ka1Moter_Step(1,equipment_parament_4);
////4.测试方向向后复位。
//	Ka1_InitBack(MotorY_speed);
////5.横移方向回到原点。	
//	Ka2_InitBack(MotorX_speed);
//}

///**
//*   @brief: 移动到指定通道的指定位置
//*   @param:  n:当前要测试的孔位 flag-> 0：扫条码位置 1:电压扫描位置 
//*   @return: void
//*   @author Hang
//*   @date: 
//*/
//void Moter_MoveToChannel(u8 n,u8 flag){
//	if(n!=Channel_now){
//		if(n>6) n=6;
//	
//	int move_ToXStep=equipment_parament_2+(n-1)*equipment_parament_1; //横移距离
//	if(n>3) move_ToXStep=move_ToXStep+(n-3)*40; //横移偏补偿,使镜头针对卡中心。
//		Ka2Moter_Step(01,move_ToXStep);   //横移方向
//	}
//	if(flag==0){
//		int moveToYStep=equipment_parament_5;  //测试方向
//		Ka1Moter_Step(01,moveToYStep);
//	}else if(flag==1){  
//		int moveToYStep=equipment_parament_6;  
//		Ka1Moter_Step(01,moveToYStep);	//测试方向	
//	}
//	Channel_now=n;
//}

