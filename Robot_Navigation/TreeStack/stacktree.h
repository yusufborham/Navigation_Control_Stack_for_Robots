# ifndef STACKTREE_H
# define STACKTREE_H


#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include "../TreeFoof/tree.h"

#define CAPACITY 362880

typedef struct {
    Node *data[CAPACITY];
    int top;
} Stack;

void initStack(Stack *s);

bool isEmpty(const Stack *s);

bool isFull(const Stack *s);

bool push(Stack *s, Node *value);

bool pop(Stack *s, Node **result);

bool peek(const Stack *s, Node **result);



#endif