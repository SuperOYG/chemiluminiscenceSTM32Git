#ifndef __Moter2_H
#define	__Moter2_H

#include "main.h"
#include "work_interface.h"

#define Moter2_TRUE 1
#define Moter2_FALSE 0

//系统状态
struct GLOBAL_FLAGS2 {
    unsigned char running:1;  		//当步进电机正在运行时，值为1
    unsigned char cmd:1;  				//当串口接收到数据时，值为1
    unsigned char out_ena:1;  		//当驱动器正常输出时,值为1
};

extern struct GLOBAL_FLAGS2 status2;

#define T1_FREQ 1000000     //定时器频率
#define FSPR    200         //步进电机单圈步数
#define SPR     (FSPR*100)  //100细分的步数
// 数学常数。 用于MSD_Move函数的简化计算
#define ALPHA (2*3.14159/SPR)                    // 2*pi/spr
#define A_T_x100 ((long)(ALPHA*T1_FREQ*100))     // (ALPHA / T1_FREQ)*100
#define T1_FREQ_148 ((int)((T1_FREQ*0.676)/100)) // divided by 100 and scaled by 0.676
#define A_SQ (long)(ALPHA*2*10000000000)         // 
#define A_x20000 (int)(ALPHA*20000)              // ALPHA*20000

//速度曲线状态
#define Moter2_STOP  0
#define Moter2_ACCEL 1
#define Moter2_DECEL 2
#define Moter2_RUN   3

typedef struct {
    unsigned char run_state : 3;  	//电机运行状态
    unsigned char dir : 1;  				//电机运行方向
    unsigned int step_delay;  			//下一个脉冲延时周期，启动时为加速度速率
    unsigned int decel_start;  			//开始减速的位置
    signed int decel_val;  					//减速距离
    signed int min_delay;  					//最小延时（即最大速度）
    signed int accel_count;  				//加速或者减速计数器
} speedRampData2;

void Moter2_Run(u8 a0,u8 a1,u32 a2);
void TIM3_Fun(void);
#endif
