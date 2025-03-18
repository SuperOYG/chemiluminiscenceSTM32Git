#include "motor.h"
#include "main.h"
#include "eeprom.h"
#include "Stdio.h"
#include "S_Moter1.h"
#include "S_Moter2.h"
#include "S_Moter3.h"

extern volatile uint32_t time;


extern u8 testStatus;

extern u32 moter3_now;      //Z轴电机当前位置
extern u32 moter3_next;     //Z轴电机下一步位置

extern u8 motor3Flag; //  Z轴电机复位标志位

extern u32 moter1_now;      //x轴电机当前位置
extern u32 moter1_next;     //x轴电机下一步位置

extern u32 moter2_now;      //y轴电机当前位置
extern u32 moter2_next;     //y轴电机下一步位置

extern u8 motor1Flag; //  x轴电机复位标志位




/**
*   @brief: 电机初始化，电机位置归零
*   @param:  void
*   @return: void
*   @author Hang
*   @date:
*/
void Motor_init()
{
    moter2_int(200); //振动电机初始化
}


/***************   带加减速控制电机1**************/
void moter1_1step(u8 speed)
{
    Delay_moter3(speed);
    CP1_TOGGLE;
}

void moter1_step(u32 distance,u8 flag)//distance = 坐标;
{
    u32 step;
    moter1_next = distance;

    if(moter1_next > moter1_now)
    {
        DIR1_H;
        step = moter1_next - moter1_now;
    }
    else if(moter1_next < moter1_now)
    {
        DIR1_L;
        step = moter1_now - moter1_next;
    }
    else
    {
        step = 0;
    }
//    printf("step 的大小为：%d",step);
    if(step > 500)
    {
        Moter1_Run(flag,0,step);
    }
    if(step == 500)
    {
        Moter1_Run(flag,1,step);
    }
    if((step > 0)&&(step < 500))
    {
        Moter1_Run(flag,2,step);
    }
    if(step == 0)
    {;}
    moter1_now = moter1_next;
}

//电机1 复位：

void moter1_int(u16 step) {
    DIR1_L;
    Moter1_Run(1,2,20000);  //10000：为运动最远处的步数，待定。
    DIR1_H;
    moter1_now=0;
    HAL_Delay(200);
    Moter1_Run(0,1,step); //碰到开关，回拨一小步。
    moter1_now=0;
}



/***************   带加减速控制电机2**************/
void moter2_1step(u8 speed)
{
    Delay_moter3(speed);
    CP2_TOGGLE;
}

void moter2_step(u32 distance,u8 flag)//distance = 坐标;
{
//	motor1Flag=0;
    u32 step;
    moter2_next = distance;

    if(moter2_next > moter2_now)
    {
        DIR2_H;
        step = moter2_next - moter2_now;
    }
    else if(moter2_next < moter2_now)
    {
        DIR2_L;
        step = moter2_now - moter2_next;
    }
    else
    {
        step = 0;
    }
    printf("step 的大小为：%d",step);
    if(step > 500)
    {
        Moter2_Run(flag,2,step);
    }
    if(step == 500)
    {
        Moter2_Run(flag,2,step);
    }
    if((step > 0)&&(step < 500))
    {
        Moter2_Run(flag,2,step);
    }
    if(step == 0)
    {;}
    moter2_now = moter2_next;
    I2C_write2Byte(equipment_addres_1, moter2_now);
}

void moter2_step2(u32 distance,u8 flag)//distance = 坐标;
{
//	motor1Flag=0;
    u32 step;
    moter2_next = distance;

    if(moter2_next > moter2_now)
    {
        DIR2_H;
        step = moter2_next - moter2_now;
    }
    else if(moter2_next < moter2_now)
    {
        DIR2_L;
        step = moter2_now - moter2_next;
    }
    else
    {
        step = 0;
    }
    printf("step 的大小为：%d",step);
    if(step > 500)
    {
        Moter2_Run(flag,2,step);
    }
    if(step == 500)
    {
        Moter2_Run(flag,2,step);
    }
    if((step > 0)&&(step < 500))
    {
        Moter2_Run(flag,2,step);
    }
    if(step == 0)
    {;}
    moter2_now = moter2_next;
    I2C_write2Byte(equipment_addres_1, moter2_now);
}



void moter2_int(u16 step) {
    DIR2_L;
    Moter2_Run(1,2,27500);  //27500：为运动最远处的步数，待定。
    DIR2_H;
    moter2_now=0;
    Moter2_Run(0,2,step); //碰到开关，回拨一小步。
    moter2_now=step;
}


/***************   带加减速控制电机3***************/
void Delay_moter3(u16 n)
{
    unsigned int i;
    for(; n>0; n--) {
        for(i=100; i>0; i--);
    }
}

void moter3_1step(u8 speed)
{
    Delay_moter3(speed);
    CP3_TOGGLE;
}

void moter3_step(u32 distance,u8 flag)//distance = 坐标;
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
    {
        step = 0;
    }
    printf("step 的大小为：%d",step);
    if(step > 500)
    {
        Moter3_Run(flag,2,step);
    }
    if(step == 500)
    {
        Moter3_Run(flag,2,step);
    }
    if((step > 0)&&(step < 500))
    {
        Moter3_Run(flag,2,step);
    }
    if(step == 0)
    {;}
    moter3_now = moter3_next;
    //1.将当前Z轴电机写入eeprom;
//  I2C_write2Byte(equipment_addres_7, moter3_now);
}

void moter3_int(u16 step) {
    DIR3_L;
    Moter3_Run(1,2,20000);  //10000：为运动最远处的步数，待定。
    DIR3_H;
    HAL_Delay(500);
    moter3_now=0;
    Moter3_Run(0,1,step); //碰到开关，回拨一小步。
    moter3_now=step;
}
