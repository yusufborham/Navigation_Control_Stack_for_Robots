#ifndef TREE_H
#define TREE_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_CHILDREN 4
#define MAX_STATE_LENGTH 10

typedef struct Node {
    char isRoot;
    char state[MAX_STATE_LENGTH];
    struct Node *children[MAX_CHILDREN];
    struct Node *parent;
    long ID ;
    int childCount;
    double priority; 
    int cost ; 
} Node;

Node *createNode(char *state, char isRoot);
Node *addChild(Node *parent, char *state);
Node *getParent(Node *child);

void printTree(const Node *node, int depth);
void freeTree(Node *node);
void deleteNChild(Node *node , int N);

#endif