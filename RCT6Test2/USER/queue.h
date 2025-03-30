#ifndef __QUEUE_H
#define __QUEUE_H
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "main.h"
#include "string.h"
#define QUEUE_SIZE 10     // 消息队列容量
#define MAX_FRAME_LEN 64  // 最大帧长度

typedef struct {
    uint8_t data[MAX_FRAME_LEN];
    uint8_t length;
} Frame;

typedef struct {
    Frame frames[QUEUE_SIZE];
    uint8_t front;  // 队列头
    uint8_t rear;   // 队列尾
    uint8_t count;  // 当前队列中消息数量
} MessageQueue;


void Queue_Init(MessageQueue *q);
uint8_t Queue_Push(MessageQueue *q, const Frame *frame);
uint8_t Queue_Pop(MessageQueue *q, Frame *frame);

#endif
