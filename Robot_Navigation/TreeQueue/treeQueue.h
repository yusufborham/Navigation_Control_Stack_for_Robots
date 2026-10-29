#ifndef TREE_QUEUE_H
#define TREE_QUEUE_H

#include <stdbool.h>
#include "../TreeFoof/tree.h"

#define QUEUE_CAPACITY 362880

typedef struct {
    Node *data[QUEUE_CAPACITY];

    int front;  // Index of the next node to remove
    int rear;   // Index where the next node will be inserted
    int count;  // Number of nodes currently stored
} Queue;

void initQueue(Queue *q);

bool isQueueEmpty(const Queue *q);
bool isQueueFull(const Queue *q);

bool enqueue(Queue *q, Node *node);
bool dequeue(Queue *q, Node **result);
bool peekQueue(const Queue *q, Node **result);

#endif