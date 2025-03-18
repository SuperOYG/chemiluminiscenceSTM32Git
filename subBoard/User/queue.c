#include "queue.h"
int InitQueue(QUEUE *Q)
{
	Q->front = 0;
	Q->rear = 0;
	return 1;
}
int ClearQueue(QUEUE *Q)
{
	Q->front = 0;
	Q->rear = 0;
	return 1;
}
int QueueLength(const QUEUE *Q)
{
	return ((Q->rear - Q->front + QUEUE_MAX_SIZE) % QUEUE_MAX_SIZE);
}
int Enqueue(QUEUE *Q,char *e)
{
	if((Q->rear+1) % QUEUE_MAX_SIZE == Q->front)
	{
		return 0;
	}
	int i=0;
	for(i=0;i<12;i++)
	{
		Q->Queue[Q->rear][i] = e[i];
	}
	Q->rear = (Q->rear + 1) % QUEUE_MAX_SIZE;
	return 1;
}
int Dequeue(QUEUE *Q,char *e)
{
	if(Q->front == Q->rear)
	{
		return 0;
	}
	int i=0;
	for(i=0;i<12;i++)
	{
		e[i] = Q->Queue[Q->front][i];
	}
	Q->front = (Q->front + 1) % QUEUE_MAX_SIZE;
	return 1;
}
int GetQueueFirstData(QUEUE *Q,char *e)
{
	if(Q->front == Q->rear)
	{
		return 0;
	}
	int i=0;
	for(i=0;i<12;i++)
	{
		e[i] = Q->Queue[Q->front][i];	
	}
	return 1;
}
