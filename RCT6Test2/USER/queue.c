#include "queue.h"


MessageQueue msgQueue;  // ȫ����Ϣ����
// ��ʼ������
void Queue_Init(MessageQueue *q) {
    q->front = 0;
    q->rear = 0;
    q->count = 0;
}

// ��Ӳ��������ж��е��ã��豣֤ԭ���ԣ�
uint8_t Queue_Push(MessageQueue *q, const Frame *frame) {
    if (q->count >= QUEUE_SIZE) return 0;  // ������

    memcpy(&q->frames[q->rear], frame, sizeof(Frame));
    q->rear = (q->rear + 1) % QUEUE_SIZE;
    q->count++;
    return 1;
}

// ���Ӳ���������ѭ���е��ã�
uint8_t Queue_Pop(MessageQueue *q, Frame *frame) {
    if (q->count == 0) return 0;  // ���п�

    memcpy(frame, &q->frames[q->front], sizeof(Frame));
    q->front = (q->front + 1) % QUEUE_SIZE;
    q->count--;
    return 1;
}