#ifndef __Moter1_H
#define	__Moter1_H

//#include "stm32f10x.h"
#include "main.h"


#define Moter1_TRUE 1
#define Moter1_FALSE 0

//ϵͳ״̬
struct GLOBAL_FLAGS1 {
  unsigned char running:1;  		//�����������������ʱ��ֵΪ1
  unsigned char cmd:1;  				//�����ڽ��յ�����ʱ��ֵΪ1
  unsigned char out_ena:1;  		//���������������ʱ,ֵΪ1
};

extern struct GLOBAL_FLAGS1 status1;

#define T1_FREQ 1000000     //��ʱ��Ƶ��
#define FSPR    200         //���������Ȧ����
#define SPR     (FSPR*100)  //100ϸ�ֵĲ���
// ��ѧ������ ����MSD_Move�����ļ򻯼���
#define ALPHA (2*3.14159/SPR)                    // 2*pi/spr
#define A_T_x100 ((long)(ALPHA*T1_FREQ*100))     // (ALPHA / T1_FREQ)*100
#define T1_FREQ_148 ((int)((T1_FREQ*0.676)/100)) // divided by 100 and scaled by 0.676
#define A_SQ (long)(ALPHA*2*10000000000)         // 
#define A_x20000 (int)(ALPHA*20000)              // ALPHA*20000
    
//�ٶ�����״̬
#define Moter1_STOP  0
#define Moter1_ACCEL 1
#define Moter1_DECEL 2
#define Moter1_RUN   3

typedef struct {
  unsigned char run_state : 3;  	//�������״̬
  unsigned char dir : 1;  				//������з���
  unsigned int step_delay;  			//��һ��������ʱ���ڣ�����ʱΪ���ٶ�����
  unsigned int decel_start;  			//��ʼ���ٵ�λ��
  signed int decel_val;  					//���پ���
  signed int min_delay;  					//��С��ʱ��������ٶȣ�
  signed int accel_count;  				//���ٻ��߼��ټ�����
} speedRampData1;

void Moter1_Run(uint8_t a0,uint8_t a1,uint32_t a2);


void TIM4Fun(void);

#endif 
