
#include "S_Moter3.h"
#include <stdio.h>
#include <math.h>
#include "motor.h"

unsigned int Moter3_canshu[][3]={
		{800,800,500},    
		{150,150,200},   
		{100,100,100}, 																	
          };

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;		  
extern uint8_t motor3Flag;		  
		  
u8 SingleChange3;    //通过该变量确定要查找的光电开关
speedRampData3 srd3;         //系统加减速参数
//系统电机、串口状态
struct GLOBAL_FLAGS3 status3 = {Moter3_FALSE, Moter3_FALSE,Moter3_TRUE};

void Moter3_Move(signed int step, unsigned int accel, unsigned int decel, unsigned int speed)
{
    unsigned int max_s_lim; //达到最大速度时的步数
    unsigned int accel_lim;//必须开始减速的步数(如果还没加速到达最大速度时)
    if(step == 1)  // 如果只移动一步
			{
				srd3.accel_count = -1; 												// 只移动一步
				srd3.run_state = Moter3_DECEL;								// 减速状态
				srd3.step_delay = 1000;												// 短延时
				status3.running = Moter3_TRUE;								// 配置电机为运行状态
//				TIM_SetAutoreload(TIM3,20);		//设置定时器重装值	
		        __HAL_TIM_SetAutoreload(&htim3, 20);  //--2
			 }
    else if(step != 0)// 步数不为零才移动
    {
			srd3.min_delay = A_T_x100 / speed;
			srd3.step_delay = (T1_FREQ_148 * sqrt(A_SQ / accel))/100;
			max_s_lim = (long)speed*speed/(long)(((long)A_x20000*accel)/100);
			if(max_s_lim == 0){max_s_lim = 1;}
			accel_lim = ((long)step*decel) / (accel+decel);
			if(accel_lim == 0){accel_lim = 1;}
			if(accel_lim <= max_s_lim)
				{srd3.decel_val = accel_lim - step;}
			else
				{srd3.decel_val = -(long)(max_s_lim*accel/decel);}
			if(srd3.decel_val == 0){srd3.decel_val = -1;}
			srd3.decel_start = step + srd3.decel_val;

			if(srd3.step_delay <= srd3.min_delay)
				{
						srd3.step_delay = srd3.min_delay;
						srd3.run_state = Moter3_RUN;
				}
			else
				{
						srd3.run_state = Moter3_ACCEL;
				}
			srd3.accel_count = 0;
			status3.running = Moter3_TRUE;
//			TIM_SetAutoreload(TIM3,20);//设置定时器重装值	
			__HAL_TIM_SetAutoreload(&htim3, 20);  //--2	
    }
}
 

//void TIM3_IRQHandler(void)
void TIM3Fun()
{		
  unsigned int new_step_delay;  					// 保存下一个延时周期
  static int last_accel_delay;  					// 加速过程中最后一次延时.
  static unsigned int step_count = 0;  		// 移动步数计数器
  static signed int rest = 0;  						// 记录new_step_delay中的余数，提高下一步计算的精度
	unsigned char a;
	    if ((SingleChange3 == 1)&&(motor3Flag==1))
        {
//			if(HAL_GPIO_ReadPin(IN4_GPIO_Port,IN4_Pin) == KEY_OFF &&(motor3Flag ==1))// 碰到开关.
//            {
//                status1.out_ena = 0;
//				 printf("碰到开关");
//				 EN3_L;
//            }
			 a = HAL_GPIO_ReadPin(IN4_GPIO_Port,IN4_Pin);
            if (a == KEY_OFF)
            {
                status3.out_ena = 0;
				 
            }
        }
			TIM3->CCR3=srd3.step_delay >> 1;//周期的一半	//设置占空比为50%	
			TIM3->ARR=srd3.step_delay;
				//如果禁止输出，电机则停止运动
			if(status3.out_ena != Moter3_TRUE)
				{
						srd3.run_state = Moter3_STOP;
				}
			switch(srd3.run_state) 
				{
					case Moter3_STOP:
						step_count = 0;
						rest = 0;
						TIM3->CCER &= ~(1<<7);  //禁止输出
//						TIM_Cmd(TIM3, DISABLE);
						HAL_TIM_Base_Stop_IT(&htim3);
					
						EN3_L;//关闭使能
						status3.running = Moter3_FALSE;
						break;
					case Moter3_ACCEL:
						TIM3->CCER |= 1<<7;     //使能输出
						step_count++;
						srd3.accel_count++;
						new_step_delay = srd3.step_delay - (((2 * (long)srd3.step_delay) + rest)/(4 * srd3.accel_count + 1));
						rest = ((2 * (long)srd3.step_delay)+rest)%(4 * srd3.accel_count + 1);
						//检查是够应该开始减速
						if(step_count >= srd3.decel_start) 
							{
								srd3.accel_count = srd3.decel_val;
								srd3.run_state = Moter3_DECEL;
							}
						//检查是否到达期望的最大速度
						else if(new_step_delay <= srd3.min_delay) 
							{
								last_accel_delay = new_step_delay;
								new_step_delay = srd3.min_delay;
								rest = 0;
								srd3.run_state = Moter3_RUN;
							}
						break;
					case Moter3_RUN:
						TIM3->CCER |= 1<<7; //使能输出
						step_count++;
						new_step_delay = srd3.min_delay;
						//检查是否需要开始减速
						if(step_count >= srd3.decel_start)
							{
								srd3.accel_count = srd3.decel_val;
								//以最后一次加速的延时作为开始减速的延时
								new_step_delay = last_accel_delay;
								srd3.run_state = Moter3_DECEL;
							}
						break;
					case Moter3_DECEL:
						TIM3->CCER |= 1<<7; //使能输出
						step_count++;
						srd3.accel_count++;
						new_step_delay = srd3.step_delay - (((2 * (long)srd3.step_delay) + rest)/(4 * srd3.accel_count + 1));
						rest = ((2 * (long)srd3.step_delay)+rest)%(4 * srd3.accel_count + 1);
						//检查是否为最后一步
						if(srd3.accel_count >= 0)
							{srd3.run_state = Moter3_STOP;}
						break;
				}
			  srd3.step_delay = new_step_delay;
	
}

void Moter3_Run(u8 a0,u8 a1,u32 a2)
{
	u32 acceleration,deceleration,speed,steps;
	acceleration = Moter3_canshu[a1][0];
	deceleration = Moter3_canshu[a1][1];
	speed = Moter3_canshu[a1][2];
	
	steps	= a2;    //默认移动步数    
	
	status3.running = Moter3_TRUE;
	status3.out_ena = Moter3_TRUE;
	
    if(a0 == 0)      {SingleChange3 = 0;}
	else if(a0 == 1) {SingleChange3 = 1;}
	else ;
  
	Moter3_Move(steps, acceleration, deceleration, speed);
	
//	TIM_Cmd(TIM4, ENABLE); 
	HAL_TIM_Base_Start_IT(&htim3);
	if(status3.running == Moter3_TRUE)
		{
			while(status3.running == Moter3_TRUE)
				{
					HAL_Delay(10);
					if(status3.out_ena != Moter3_TRUE)
					break;
				}
		}
}