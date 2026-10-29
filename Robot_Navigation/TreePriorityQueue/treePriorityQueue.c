#include "treePriorityQueue.h"

#include <stdint.h>
#include <stdlib.h>

bool initPriorityQueue(PriorityQueue *q, int capacity)
{
    q->data = NULL;
    q->count = 0;
    q->capacity = 0;

    if (capacity <= 0 || (size_t)capacity > SIZE_MAX / sizeof *q->data)
        return false;

    q->data = malloc((size_t)capacity * sizeof *q->data);
    if (q->data == NULL)
        return false;

    q->capacity = capacity;
    return true;
}

bool isPriorityQueueEmpty(const PriorityQueue *q)
{
    return q->count == 0;
}

bool pushPriorityQueue(PriorityQueue *q, Node *node)
{
    if (node == NULL || q->count >= q->capacity)
        return false;

    int i = q->count;
    q->data[i] = node;
    q->count++;

    /* Sift the new node upward while its parent has a larger priority. */
    while (i > 0) {
        int parent = (i - 1) / 2;

        if (q->data[parent]->priority <= q->data[i]->priority)
            break;

        Node *temp = q->data[parent];
        q->data[parent] = q->data[i];
        q->data[i] = temp;
        i = parent;
    }

    return true;
}

bool decreasePriorityQueue(PriorityQueue *q, Node *node, int newPriority)
{
    if (node == NULL || newPriority > node->priority)
        return false;

    int i = -1;

    /* Find the existing pointer. Repeated pointers would share one priority,
     * so reject that case rather than repair only one of their positions.
     */
    for (int k = 0; k < q->count; k++) {
        if (q->data[k] == node) {
            if (i != -1)
                return false;
            i = k;
        }
    }

    if (i == -1)
        return false;

    node->priority = newPriority;

    /* A decrease can only violate the order toward the root. */
    while (i > 0) {
        int parent = (i - 1) / 2;

        if (q->data[parent]->priority <= q->data[i]->priority)
            break;

        Node *temp = q->data[parent];
        q->data[parent] = q->data[i];
        q->data[i] = temp;
        i = parent;
    }

    return true;
}

bool popPriorityQueue(PriorityQueue *q, Node **result)
{
    if (result == NULL || isPriorityQueueEmpty(q))
        return false;

    *result = q->data[0];

    /* Replace the root with the last entry, then discard the last slot. */
    q->count--;
    q->data[0] = q->data[q->count];
    q->data[q->count] = NULL;

    int i = 0;

    /* Only indices below count / 2 can have children. */
    while (i < q->count / 2) {
        int left = 2 * i + 1;
        int right = left + 1;
        int smaller = left;

        if (right < q->count &&
            q->data[right]->priority < q->data[left]->priority)
            smaller = right;

        if (q->data[i]->priority <= q->data[smaller]->priority)
            break;

        Node *temp = q->data[i];
        q->data[i] = q->data[smaller];
        q->data[smaller] = temp;
        i = smaller;
    }

    return true;
}

bool peekPriorityQueue(const PriorityQueue *q, Node **result)
{
    if (result == NULL || isPriorityQueueEmpty(q))
        return false;

    *result = q->data[0];
    return true;
}

void freePriorityQueue(PriorityQueue *q)
{
    free(q->data);
    q->data = NULL;
    q->count = 0;
    q->capacity = 0;
}
