#ifndef __QUEUE_H
#define __QUEUE_H
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "main.h"
#include "string.h"
#define QUEUE_SIZE 10     // ��Ϣ��������
#define MAX_FRAME_LEN 64  // ���֡����

typedef struct {
    uint8_t data[MAX_FRAME_LEN];
    uint8_t length;
} Frame;

typedef struct {
    Frame frames[QUEUE_SIZE];
    uint8_t front;  // ����ͷ
    uint8_t rear;   // ����β
    uint8_t count;  // ��ǰ��������Ϣ����
} MessageQueue;


void Queue_Init(MessageQueue *q);
uint8_t Queue_Push(MessageQueue *q, const Frame *frame);
uint8_t Queue_Pop(MessageQueue *q, Frame *frame);

#endif
