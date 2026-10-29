#ifndef TREE_PRIORITY_QUEUE_H
#define TREE_PRIORITY_QUEUE_H

#include <stdbool.h>
#include "../TreeFoof/tree.h"


typedef struct {
    Node **data;  /* Array containing the min-heap. */
    int count;
    int capacity;
} PriorityQueue;


bool initPriorityQueue(PriorityQueue *q, int capacity);
bool isPriorityQueueEmpty(const PriorityQueue *q);

bool pushPriorityQueue(PriorityQueue *q, Node *node);


bool decreasePriorityQueue(PriorityQueue *q, Node *node, int newPriority);

bool popPriorityQueue(PriorityQueue *q, Node **result);
bool peekPriorityQueue(const PriorityQueue *q, Node **result);

void freePriorityQueue(PriorityQueue *q);

#endif
