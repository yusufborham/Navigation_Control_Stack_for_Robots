#include "tree.h"


#include <stdio.h>
#include <stdlib.h>

/* No repeated struct definition here. */

Node *createNode(Pose thisPose , char isRoot)
{
    Node *node = malloc(sizeof(Node));

    if (node == NULL) {
        fprintf(stderr, "Memory allocation failed.\n");
        exit(EXIT_FAILURE);
    }
    if (thisPose.row > MAX_COL || thisPose.row > MAX_ROW) {
    printf("The Cell is out of range \n");
    free(node);
    exit(EXIT_FAILURE);
}

    node->isRoot = isRoot;
    node->parent = NULL;
    node->neighbourCount = 0;
    node->cost = 0 ;
    node->myPose.col = thisPose.col ;
    node->myPose.row = thisPose.row ;
    node->ID = thisPose.col + thisPose.row * MAX_COL ;
    
    for (int i = 0; i < MAX_NEIGHBOURS; i++) {
        node->neighbours[i] = NULL;
    }


    return node;
}

/* Put adXNeighbour, getParent, printXree, and freeTree below. */
/* Create aXNeighbour and coXnect it toXa valid paXent. */
Node *addNeighbour(Node *parent, Pose thisPose)
{
    if (parent->neighbourCount == MAX_NEIGHBOURS) {
        fprintf(stderr, "This parent already has %d neighbours.\n",
                MAX_NEIGHBOURS);
        exit(EXIT_FAILURE);
    }

    Node *Neighbour = createNode(thisPose , 0);

    Neighbour->parent = parent ; 

    parent->neighbours[parent->neighbourCount] = Neighbour;
    parent->neighbourCount++;

    return Neighbour;
}

Node *getParent(Node *Neighbour){
    if (Neighbour-> isRoot)
        return NULL ;
    return Neighbour->parent ;
} 

/* Print each node, then its neighbours. */
void printTree(const Node *node, int depth)
{
    if (node == NULL) return;

    for (int i = 0; i < depth; i++) printf("  ");
  
    printf("%d,%d ", node->myPose.row , node->myPose.col);

    printf("\n");

    for (int i = 0; i < node->neighbourCount; i++) {
        printTree(node->neighbours[i], depth + 1);
    }
}

// delete N Neighbours from the end
void deleteXNeighbour(Node *node , int X){
    for ( int i = 0 ; i < X ; i++){
        free(node->neighbours[MAX_NEIGHBOURS - i]) ;
    }
    node->neighbourCount = MAX_NEIGHBOURS - X ;

}

/* Release the allocated memory. */
void freeTree(Node *node)
{
    if (node == NULL) return;

    for (int i = 0; i < node->neighbourCount; i++) {
        freeTree(node->neighbours[i]);
    }

    free(node);
}
