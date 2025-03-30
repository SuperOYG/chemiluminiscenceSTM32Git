#include "queue.h"


MessageQueue msgQueue;  // 全局消息队列
// 初始化队列
void Queue_Init(MessageQueue *q) {
    q->front = 0;
    q->rear = 0;
    q->count = 0;
}

// 入队操作（在中断中调用，需保证原子性）
uint8_t Queue_Push(MessageQueue *q, const Frame *frame) {
    if (q->count >= QUEUE_SIZE) return 0;  // 队列满

    memcpy(&q->frames[q->rear], frame, sizeof(Frame));
    q->rear = (q->rear + 1) % QUEUE_SIZE;
    q->count++;
    return 1;
}

// 出队操作（在主循环中调用）
uint8_t Queue_Pop(MessageQueue *q, Frame *frame) {
    if (q->count == 0) return 0;  // 队列空

    memcpy(frame, &q->frames[q->front], sizeof(Frame));
    q->front = (q->front + 1) % QUEUE_SIZE;
    q->count--;
    return 1;
}