#include "stacktree.h"


void initStack(Stack *s)
{
    s->top = -1;   // -1 means empty
}

bool isEmpty(const Stack *s)
{
    return s->top == -1;
}

bool isFull(const Stack *s)
{
    return s->top == CAPACITY - 1;
}

bool push(Stack *s, Node *value)
{
    if (isFull(s))
        return false;

    s->top++;
    s->data[s->top] = value;

    return true;
}
bool pop(Stack *s, Node **result)
{
    if (isEmpty(s))
        return false;

    *result = s->data[s->top];
    s->top--;

    return true;
}

bool peek(const Stack *s, Node **result)
{
    if (isEmpty(s))
        return false;

    *result = s->data[s->top];

    return true;
}
