#include "tree.h"


#include <stdio.h>
#include <stdlib.h>

/* No repeated struct definition here. */

Node *createNode(char *state, char isRoot)
{
    Node *node = malloc(sizeof(Node));

    if (node == NULL) {
        fprintf(stderr, "Memory allocation failed.\n");
        exit(EXIT_FAILURE);
    }
    if (strlen(state) >= MAX_STATE_LENGTH) {
    fprintf(stderr, "State is too long.\n");
    free(node);
    exit(EXIT_FAILURE);
}

    strcpy(node->state, state);


    node->isRoot = isRoot;
    node->parent = NULL;
    node->childCount = 0;
    node->cost = 0 ;
    node->state[MAX_STATE_LENGTH-1] = '\0';
    for (int i = 0; i < MAX_CHILDREN; i++) {
        node->children[i] = NULL;
    }


    return node;
}

/* Put addChild, getParent, printTree, and freeTree below. */
/* Create a child and connect it to a valid parent. */
Node *addChild(Node *parent, char *state)
{
    if (parent->childCount == MAX_CHILDREN) {
        fprintf(stderr, "This parent already has %d children.\n",
                MAX_CHILDREN);
        exit(EXIT_FAILURE);
    }

    Node *child = createNode(state , 0);

    child->parent = parent ; 

    parent->children[parent->childCount] = child;
    parent->childCount++;

    return child;
}

Node *getParent(Node *child){
    if (child-> isRoot)
        return NULL ;
    return child->parent ;
} 

/* Print each node, then its children. */
void printTree(const Node *node, int depth)
{
    if (node == NULL) return;

    for (int i = 0; i < depth; i++) printf("  ");
    for (int i = 0; i < MAX_STATE_LENGTH; i++) {
        if (node->state[i] == '\0') break;
        printf("%c", node->state[i]);
    }
    printf("\n");

    for (int i = 0; i < node->childCount; i++) {
        printTree(node->children[i], depth + 1);
    }
}

// delete N childs from the end
void deleteNChild(Node *node , int N){
    for ( int i = 0 ; i < N ; i++){
        free(node->children[MAX_CHILDREN - i]) ;
    }
    node->childCount = MAX_CHILDREN - N ;

}

/* Release the allocated memory. */
void freeTree(Node *node)
{
    if (node == NULL) return;

    for (int i = 0; i < node->childCount; i++) {
        freeTree(node->children[i]);
    }

    free(node);
}
