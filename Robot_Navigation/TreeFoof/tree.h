#ifndef TREE_H
#define TREE_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_NEIGHBOURS 4
#define MAX_STATE_LENGTH 10
#define MAX_ROW 1000
#define MAX_COL 2000

typedef struct {

    int row ; 
    int col ;

} Pose ;

typedef struct Node {
    char isRoot;
    struct Node *neighbours[MAX_NEIGHBOURS];
    struct Node *parent;
    long ID ;
    int neighbourCount;
    double priority; 
    Pose myPose ;
    int cost ; 
} Node;

Node *createNode(Pose thisPose , char isRoot);
Node *addNeighbour(Node *parent, Pose thisPose);
Node *getParent(Node *Neighbour);

void printTree(const Node *node, int depth);
void freeTree(Node *node);
void deleteXNeighbour(Node *node , int X);

#endif