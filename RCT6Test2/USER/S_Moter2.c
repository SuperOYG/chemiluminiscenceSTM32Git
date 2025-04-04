#include "S_Moter2.h"
#include <stdio.h>
#include <math.h>
#include "motor.h"


//电机2：对应的是TIM3_CH1  ->PA7 TIM3_CH2

unsigned int Moter2_canshu[][3] =
{
    {800,800,500},
    {150,150,200},
    {50,50,100}
};
u8 SingleChange2;    //通过该变量确定要查找的光电开关
speedRampData2 srd2;         //系统加减速参数
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern uint8_t motor2Flag;
//uint8_t motor2Flag;




struct GLOBAL_FLAGS2 status2 = {Moter2_FALSE, Moter2_FALSE, Moter2_TRUE};

void Moter2_Move(signed int step, unsigned int accel, unsigned int decel, unsigned int speed)
{

    unsigned int max_s_lim; //达到最大速度时的步数
    unsigned int accel_lim;//必须开始减速的步数(如果还没加速到达最大速度时)
    if (step == 1) // 如果只移动一步
    {
        srd2.accel_count = -1;                                              // 只移动一步
        srd2.run_state = Moter2_DECEL;                              // 减速状态
        srd2.step_delay = 1000;                                             // 短延时
        status2.running = Moter2_TRUE;                              // 配置电机为运行状态
//      TIM_SetAutoreload(TIM4,20);     //设置定时器重装值
        __HAL_TIM_SetAutoreload(&htim3, 20);  //--2
    }
    else if (step != 0) // 步数不为零才移动
    {
        srd2.min_delay = A_T_x100 / speed;
        srd2.step_delay = (T1_FREQ_148 * sqrt(A_SQ / accel)) / 100;
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
            srd2.decel_val = accel_lim - step;
        }
        else
        {
            srd2.decel_val = -(long)(max_s_lim * accel / decel);
        }
        if (srd2.decel_val == 0)
        {
            srd2.decel_val = -1;
        }
        srd2.decel_start = step + srd2.decel_val;

        if (srd2.step_delay <= srd2.min_delay)
        {
            srd2.step_delay = srd2.min_delay;
            srd2.run_state = Moter2_RUN;
        }
        else
        {
            srd2.run_state = Moter2_ACCEL;
        }
        srd2.accel_count = 0;
        status2.running = Moter2_TRUE;
//          TIM_SetAutoreload(TIM4,20);//设置定时器重装值
        __HAL_TIM_SetAutoreload(&htim3, 20);  //--2
    }
}

void TIM3_Fun(void)
{
    unsigned int new_step_delay;                      // 保存下一个延时周期
    static int last_accel_delay;                      // 加速过程中最后一次延时.
    static unsigned int step_count = 0;       // 移动步数计数器
    static signed int rest = 0;                       // 记录new_step_delay中的余数，提高下一步计算的精度
    unsigned char a;
    if (SingleChange2 == 1)
    {
        a = HAL_GPIO_ReadPin(IN3_GPIO_Port, IN3_Pin);
        if (a == KEY_OFF)
        {
            status2.out_ena = 0;
            motor2Flag=1;
        }
    }
    else ;

    TIM3->CCR2 = srd2.step_delay >> 1; //周期的一半    //设置占空比为50%
    TIM3->ARR = srd2.step_delay;

    //如果禁止输出，电机则停止运动
    if (status2.out_ena != Moter2_TRUE)
    {
        srd2.run_state = Moter2_STOP;
    }
    switch (srd2.run_state)
    {
    case Moter2_STOP:
        step_count = 0;
        rest = 0;
        TIM3->CCER &= ~(1 << 4); //禁止输出
//                      TIM_Cmd(TIM4, DISABLE);

        HAL_TIM_Base_Stop_IT(&htim3); //--2
        status2.running = Moter2_FALSE;
        break;
    case Moter2_ACCEL:
        TIM3->CCER |= 1 << 4;   //使能输出
        step_count++;
        srd2.accel_count++;
        new_step_delay = srd2.step_delay - (((2 * (long)srd2.step_delay) + rest) / (4 * srd2.accel_count + 1));
        rest = ((2 * (long)srd2.step_delay) + rest) % (4 * srd2.accel_count + 1);
        //检查是够应该开始减速
        if (step_count >= srd2.decel_start)
        {
            srd2.accel_count = srd2.decel_val;
            srd2.run_state = Moter2_DECEL;
        }
        //检查是否到达期望的最大速度
        else if (new_step_delay <= srd2.min_delay)
        {
            last_accel_delay = new_step_delay;
            new_step_delay = srd2.min_delay;
            rest = 0;
            srd2.run_state = Moter2_RUN;
        }
        break;
    case Moter2_RUN:
        TIM3->CCER |= 1 << 4; //使能输出
        step_count++;
        new_step_delay = srd2.min_delay;
        //检查是否需要开始减速
        if (step_count >= srd2.decel_start)
        {
            srd2.accel_count = srd2.decel_val;
            //以最后一次加速的延时作为开始减速的延时
            new_step_delay = last_accel_delay;
            srd2.run_state = Moter2_DECEL;
        }
        break;
    case Moter2_DECEL:
        TIM3->CCER |= 1 << 4; //使能输出
        step_count++;
        srd2.accel_count++;
        new_step_delay = srd2.step_delay - (((2 * (long)srd2.step_delay) + rest) / (4 * srd2.accel_count + 1));
        rest = ((2 * (long)srd2.step_delay) + rest) % (4 * srd2.accel_count + 1);
        //检查是否为最后一步
        if (srd2.accel_count >= 0)
        {
            srd2.run_state = Moter2_STOP;
        }
        break;
    }
    srd2.step_delay = new_step_delay;
}

void Moter2_Run(u8 a0, u8 a1, u32 a2)
{
    u32 acceleration, deceleration, speed, steps;
    acceleration = Moter2_canshu[a1][0];
    deceleration = Moter2_canshu[a1][1];
    speed = Moter2_canshu[a1][2];

    steps   = a2*Motor1_2Factor;    //默认移动步数

    status2.running = Moter2_TRUE;
    status2.out_ena = Moter2_TRUE;

    if (a0 == 0)
    {
        SingleChange2 = 0;
    }
    else if (a0 == 1)
    {
        SingleChange2 = 1;
    }
    else ;

    Moter2_Move(steps, acceleration, deceleration, speed);
    HAL_TIM_Base_Start_IT(&htim3); //--2
    if(status2.running == Moter2_TRUE)
    {
        while(status2.running == Moter2_TRUE)
        {
            HAL_Delay(10);
            if(status2.out_ena != Moter2_TRUE)
                break;
        }
    }
}
