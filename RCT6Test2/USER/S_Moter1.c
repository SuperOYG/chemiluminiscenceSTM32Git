#include "S_Moter1.h"
#include <stdio.h>
#include <math.h>
#include "motor.h"

extern TIM_HandleTypeDef htim4;
extern uint8_t motor1Flag;

u8 number;
unsigned int Moter1_canshu[][3] =
{
//    {800, 800, 500},
	   {1500, 1500, 2000},
    {500, 500, 250},
    {300, 300, 165},
    {100, 100, 100},
};

//将  通道输出改为通道TIM4_CH4;

uint8_t SingleChange1;    //通过该变量确定要查找的光电开关
speedRampData1 srd1;          //系统加减速参数

//系统电机、串口状态
struct GLOBAL_FLAGS1 status1 = {Moter1_FALSE, Moter1_FALSE, Moter1_TRUE};


void Moter1_Move(signed int step, unsigned int accel, unsigned int decel, unsigned int speed)
{
    unsigned int max_s_lim; //达到最大速度时的步数
    unsigned int accel_lim;//必须开始减速的步数(如果还没加速到达最大速度时)
    if (step == 1) // 如果只移动一步
    {
        srd1.accel_count = -1;                                        // 只移动一步
        srd1.run_state = Moter1_DECEL;                              // 减速状态
        srd1.step_delay = 1000;                                       // 短延时
        status1.running = Moter1_TRUE;                              // 配置电机为运行状态
        __HAL_TIM_SetAutoreload(&htim4, 20);  //--2
    }
    else if (step != 0) // 步数不为零才移动
    {
        srd1.min_delay = A_T_x100 / speed;
        srd1.step_delay = (T1_FREQ_148 * sqrt(A_SQ / accel)) / 100;
        max_s_lim = (long)speed * speed / (long)(((long)A_x20000 * accel) / 100);
        if (max_s_lim == 0)
        {
            max_s_lim = 1;
        }
        accel_lim = ((long)step * decel) / (accel + decel);
        if (accel_lim == 0)
        {
            accel_lim = 1;
        }
        if (accel_lim <= max_s_lim)
        {
            srd1.decel_val = accel_lim - step;
        }
        else
        {
            srd1.decel_val = -(long)(max_s_lim * accel / decel);
        }
        if (srd1.decel_val == 0)
        {
            srd1.decel_val = -1;
        }
        srd1.decel_start = step + srd1.decel_val;

        if (srd1.step_delay <= srd1.min_delay)
        {
            srd1.step_delay = srd1.min_delay;
            srd1.run_state = Moter1_RUN;
        }
        else
        {
            srd1.run_state = Moter1_ACCEL;
        }
        srd1.accel_count = 0;
        status1.running = Moter1_TRUE;
//          TIM_SetAutoreload(TIM4,20);//设置定时器重装值
        __HAL_TIM_SetAutoreload(&htim4, 20);  //--2

    }
}

void TIM4Fun(void)
{
    unsigned int new_step_delay;                      // 保存下一个延时周期
    static int last_accel_delay;                      // 加速过程中最后一次延时.
    static unsigned int step_count = 0;       // 移动步数计数器
    static signed int rest = 0;                       // 记录new_step_delay中的余数，提高下一步计算的精度
    unsigned char a;
    if (SingleChange1 == 1)
    {
        a = HAL_GPIO_ReadPin(IN7_GPIO_Port, IN7_Pin);
        if (a == KEY_OFF)
        {
            status1.out_ena = 0;
            motor1Flag=1;
        }
    }
    TIM4->CCR4 = srd1.step_delay >> 1; //周期的一半    //设置占空比为50%
    TIM4->ARR = srd1.step_delay;

    //如果禁止输出，电机则停止运动
    if (status1.out_ena != Moter1_TRUE)
    {
        srd1.run_state = Moter1_STOP;
    }
    switch (srd1.run_state)
    {
    case Moter1_STOP:
        step_count = 0;
        status1.out_ena = 0;
        rest = 0;
        TIM4->CCER &= ~(1<<12); //禁止输出
        HAL_TIM_Base_Stop_IT(&htim4);
        status1.running = Moter1_FALSE;
        break;
    case Moter1_ACCEL:
        TIM4->CCER |= 1 << 12;   //使能输出
        step_count++;
        srd1.accel_count++;
        new_step_delay = srd1.step_delay - (((2 * (long)srd1.step_delay) + rest) / (4 * srd1.accel_count + 1));
        rest = ((2 * (long)srd1.step_delay) + rest) % (4 * srd1.accel_count + 1);
        //检查是够应该开始减速
        if (step_count >= srd1.decel_start)
        {
            srd1.accel_count = srd1.decel_val;
            srd1.run_state = Moter1_DECEL;
        }
        //检查是否到达期望的最大速度
        else if (new_step_delay <= srd1.min_delay)
        {
            last_accel_delay = new_step_delay;
            new_step_delay = srd1.min_delay;
            rest = 0;
            srd1.run_state = Moter1_RUN;
        }
        break;
    case Moter1_RUN:
        TIM4->CCER |=1 << 12 ; //使能输出
        step_count++;
        new_step_delay = srd1.min_delay;
        //检查是否需要开始减速
        if (step_count >= srd1.decel_start)
        {
            srd1.accel_count = srd1.decel_val;
            //以最后一次加速的延时作为开始减速的延时
            new_step_delay = last_accel_delay;
            srd1.run_state = Moter1_DECEL;
        }
        break;
    case Moter1_DECEL:
        TIM4->CCER |= 1 << 12; //使能输出
        step_count++;
        srd1.accel_count++;
        new_step_delay = srd1.step_delay - (((2 * (long)srd1.step_delay) + rest) / (4 * srd1.accel_count + 1));
        rest = ((2 * (long)srd1.step_delay) + rest) % (4 * srd1.accel_count + 1);
        //检查是否为最后一步
        if (srd1.accel_count >= 0)
        {
            srd1.run_state = Moter1_STOP;
        }
        break;
    }
    srd1.step_delay = new_step_delay;
}

void Moter1_Run(uint8_t a0, uint8_t a1, uint32_t a2)
{
    uint32_t acceleration, deceleration, speed, steps;
    acceleration = Moter1_canshu[a1][0];
    deceleration = Moter1_canshu[a1][1];
    speed = Moter1_canshu[a1][2];

    steps   = a2*Motor1_2Factor;    //默认移动步数

    status1.running = Moter1_TRUE;
    status1.out_ena = Moter1_TRUE;
    EN1_H;
    if (a0 == 0)
    {
        SingleChange1 = 0;
    }
    else if (a0 == 1)
    {
        SingleChange1 = 1;
    }
    else ;
		
    Moter1_Move(steps, acceleration, deceleration, speed);
    HAL_TIM_Base_Start_IT(&htim4); //--2
    if(status1.running == Moter1_TRUE)
    {

        while(status1.running == Moter1_TRUE)
        {
            HAL_Delay(10);
            if(status1.out_ena != Moter1_TRUE) {
                break;
            }
        }
    }
}
