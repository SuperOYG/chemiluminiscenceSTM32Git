#ifndef __QUEUE_H
#define __QUEUE_H
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#define QUEUE_MAX_SIZE 100
#define QUEUE_CMD_SIZE  12
typedef struct
{
	char Queue[QUEUE_MAX_SIZE][QUEUE_CMD_SIZE];
	int front;
	int rear;
}QUEUE;
int InitQueue(QUEUE *Q);
int ClearQueue(QUEUE *Q);
int QueueLength(const QUEUE *Q);
int Enqueue(QUEUE *Q,char *e);
int Dequeue(QUEUE *Q,char *e);
int GetQueueFirstData(QUEUE *Q,char *e);

#endif
