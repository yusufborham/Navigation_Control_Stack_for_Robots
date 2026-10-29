#include "treeQueue.h"

void initQueue(Queue *q)
{
    q->front = 0;
    q->rear = 0;
    q->count = 0;
}

bool isQueueEmpty(const Queue *q)
{
    return q->count == 0;
}

bool isQueueFull(const Queue *q)
{
    return q->count == QUEUE_CAPACITY;
}

bool enqueue(Queue *q, Node *node)
{
    if (isQueueFull(q))
        return false;

    q->data[q->rear] = node;
    q->rear = (q->rear + 1) % QUEUE_CAPACITY;
    q->count++;

    return true;
}

bool dequeue(Queue *q, Node **result)
{
    if (isQueueEmpty(q))
        return false;

    *result = q->data[q->front];
    q->front = (q->front + 1) % QUEUE_CAPACITY;
    q->count--;

    return true;
}

bool peekQueue(const Queue *q, Node **result)
{
    if (isQueueEmpty(q))
        return false;

    *result = q->data[q->front];

    return true;
}