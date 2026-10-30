#define _POSIX_C_SOURCE 200809L /* getline(): must precede all includes. */
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <ctype.h>
#include <errno.h>
#include <stdint.h>



#include "./TreeFoof/tree.h"
#include "./TreePriorityQueue/treePriorityQueue.h"

/**
 * Read a MAX_ROW x MAX_COL CSV in row-major order.
 * Input: 0 = free, 100 = obstacle. Output: 0 = free, 1 = obstacle.
 * getline() grows its line buffer; it never splits a number at 4095 bytes.
 * Reject malformed values and dimensions instead of silently corrupting cells.
 * Caller owns the returned allocation. On failure, return NULL and size 0.
 */
char* load_occupancy_grid(const char *filename, size_t *out_size) {
    if (!out_size) return NULL;
    *out_size = 0;

    const size_t expected_rows = (size_t)MAX_ROW;
    const size_t expected_cols = (size_t)MAX_COL;
    if (!expected_rows || !expected_cols || expected_rows > SIZE_MAX / expected_cols) {
        fprintf(stderr, "Invalid or overflowing map dimensions.\n");
        return NULL;
    }
    const size_t expected_cells = expected_rows * expected_cols;
    FILE *fp = fopen(filename, "r");
    if (!fp) {
        perror(filename);
        return NULL;
    }
    char *grid = malloc(expected_cells * sizeof *grid);
    char *line = NULL;
    size_t line_capacity = 0;
    size_t rows = 0, count = 0, line_number = 0;
    if (!grid) {
        perror("Allocating occupancy grid");
        fclose(fp);
        return NULL;
    }

    while (getline(&line, &line_capacity, fp) != -1) {
        ++line_number;
        char *p = line;
        /* Accept the optional UTF-8 BOM also accepted by the Python reader. */
        if (line_number == 1 && strncmp(p, "\xEF\xBB\xBF", 3) == 0) p += 3;
        while (isspace((unsigned char)*p)) ++p;
        if (*p == '\0' || *p == '#') continue;
        if (rows >= expected_rows) {
            fprintf(stderr, "%s:%zu: more than %zu grid rows.\n",
                    filename, line_number, expected_rows);
            goto fail;
        }
        size_t cols = 0;
        for (;;) {
            errno = 0;
            char *end = NULL;
            long value = strtol(p, &end, 10);
            if (p == end || errno == ERANGE || (value != 0 && value != 100)) {
                fprintf(stderr, "%s:%zu: invalid cell at column %zu; expected 0 or 100.\n",
                        filename, line_number, cols);
                goto fail;
            }
            if (cols >= expected_cols) {
                fprintf(stderr, "%s:%zu: more than %zu columns.\n",
                        filename, line_number, expected_cols);
                goto fail;
            }
            grid[count++] = (value == 100) ? 1 : 0;
            ++cols;
            p = end;
            while (isspace((unsigned char)*p)) ++p;
            if (*p == '\0' || *p == '#') break;
            if (*p != ',') {
                fprintf(stderr, "%s:%zu: expected comma after column %zu.\n",
                        filename, line_number, cols - 1);
                goto fail;
            }
            ++p;
            while (isspace((unsigned char)*p)) ++p;
            /* A trailing comma or two consecutive commas is an empty cell. */
            if (*p == '\0' || *p == '#') {
                fprintf(stderr, "%s:%zu: empty cell after trailing comma.\n",
                        filename, line_number);
                goto fail;
            }
        }
        if (cols != expected_cols) {
            fprintf(stderr, "%s:%zu: got %zu columns; expected %zu.\n",
                    filename, line_number, cols, expected_cols);
            goto fail;
        }
        ++rows;
    }
    if (ferror(fp)) {
        fprintf(stderr, "Error reading %s.\n", filename);
        goto fail;
    }
    if (rows != expected_rows || count != expected_cells) {
        fprintf(stderr, "%s: got %zu rows and %zu cells; expected %zu rows and %zu cells.\n",
                filename, rows, count, expected_rows, expected_cells);
        goto fail;
    }
    free(line);
    fclose(fp);
    *out_size = count;
    return grid;

fail:
    free(line);
    free(grid);
    fclose(fp);
    return NULL;
}

enum {
    UP = -3 ,
    LEFT = -1 ,
    RIGHT = 1 ,
    DOWN = 3
} POSITIONS_t ;


int computeManhattan(Pose thisPose , Pose goalPose) {
    int heur = 0;
    heur = abs(thisPose.row - goalPose.row) + abs(thisPose.col - goalPose.col) ;
    return heur ;
}

  
bool astar(Node *root, Pose goalPose , char *occupicyMap) {
    PriorityQueue frontier ;

    if (!initPriorityQueue(&frontier, MAX_COL * MAX_ROW * 2))
    return false;

    Node *stateNow = root ;
    int stateCosts[MAX_COL * MAX_ROW] = {0} ; 

    stateNow->priority = computeManhattan(stateNow->myPose , goalPose) ;
    stateCosts[(int)stateNow->ID] =  0 ;  // cost to reach the root is 0

    int exploredStates = 0 ;

    while (1){  

        
        // Expand The State 
        int rowNow = stateNow->myPose.row ;
        int colNow = stateNow->myPose.col ;

        // basically we check if the neighbouring cell is within the grid, not occupied, and if the cost to reach it is less than the previously recorded cost (or if it hasn't been recorded yet)
        bool canMoveRight = (colNow != MAX_COL - 1) && !(occupicyMap[rowNow * MAX_COL + colNow + 1]) && ( ((stateNow->cost + 1) < stateCosts[rowNow * MAX_COL + colNow + 1]) || (stateCosts[rowNow * MAX_COL + colNow + 1] == 0) ) ;
        bool canMoveLeft = (colNow != 0) && !(occupicyMap[rowNow * MAX_COL + colNow - 1]) && ( ((stateNow->cost + 1) < stateCosts[rowNow * MAX_COL + colNow - 1]) || (stateCosts[rowNow * MAX_COL + colNow - 1] == 0) ) ;
        bool canMoveUp = (rowNow != 0) && !(occupicyMap[(rowNow - 1) * MAX_COL + colNow]) && ( ((stateNow->cost + 1) < stateCosts[(rowNow - 1) * MAX_COL + colNow]) || (stateCosts[(rowNow - 1) * MAX_COL + colNow] == 0) ) ;
        bool canMoveDown = (rowNow != MAX_ROW - 1) && !(occupicyMap[(rowNow + 1) * MAX_COL + colNow]) && ( ((stateNow->cost + 1) < stateCosts[(rowNow + 1) * MAX_COL + colNow]) || (stateCosts[(rowNow + 1) * MAX_COL + colNow] == 0) ) ;
        
        if(canMoveRight){
            Node *neighbour = addNeighbour(stateNow , (Pose){rowNow , colNow + 1}) ;  
            neighbour->cost = stateNow->cost + 1 ;   
            stateCosts[neighbour->ID] = neighbour->cost ;
            neighbour->priority = computeManhattan(neighbour->myPose ,goalPose) + neighbour->cost ;  // f(n) = g(n) + h(n)
            // push the new state 
            if (!pushPriorityQueue(&frontier, neighbour)) {
                freePriorityQueue(&frontier);
                return false;
            }
            
        }
        if(canMoveLeft){
            Node *neighbour = addNeighbour(stateNow , (Pose){rowNow , colNow - 1}) ; 
            neighbour->cost = stateNow->cost + 1 ;   
            stateCosts[neighbour->ID] = neighbour->cost ;
            neighbour->priority = computeManhattan(neighbour->myPose ,goalPose) + neighbour->cost ;  // f(n) = g(n) + h(n)
            // push the new state 
            if (!pushPriorityQueue(&frontier, neighbour)) {
                freePriorityQueue(&frontier);
                return false;
            }
        }
        if(canMoveUp){
            Node *neighbour = addNeighbour(stateNow , (Pose){rowNow - 1 , colNow}) ; 
            neighbour->cost = stateNow->cost + 1 ;   
            stateCosts[neighbour->ID] = neighbour->cost ;
            neighbour->priority = computeManhattan(neighbour->myPose ,goalPose) + neighbour->cost ;  // f(n) = g(n) + h(n)
            // push the new state 
            if (!pushPriorityQueue(&frontier, neighbour)) {
                freePriorityQueue(&frontier);
                return false;
            }
        }
        if(canMoveDown){
            Node *neighbour = addNeighbour(stateNow , (Pose){rowNow + 1 , colNow}) ;  
            neighbour->cost = stateNow->cost + 1 ;  
            stateCosts[neighbour->ID] = neighbour->cost ;
            neighbour->priority = computeManhattan(neighbour->myPose ,goalPose) + neighbour->cost ;  // f(n) = g(n) + h(n)
            // push the new state 
            if (!pushPriorityQueue(&frontier, neighbour)) {
                freePriorityQueue(&frontier);
                return false;
            }
        }

        // check the state 
        Node *next ;
        while (1){
            if (popPriorityQueue(&frontier, &next)) {
                // we need to check whether the poped state was popped before under a better cost so if the cost of this node is worse than the one in our 
                // array holding the cost of each state we shall discard it
                // keep looping until we pop a valid state 
                if( stateCosts[next->ID] >= next->cost  ) {
                    stateNow = next ;
                    break ;
                }
                
            }
            else {
                printf("Queue is empty. No more nodes to explore and could not find the Goal.\n");
                freePriorityQueue(&frontier);
                return false ;
            }
        }

        // printState(stateNow->state); ---> uncomment to print the explored states 
        exploredStates++ ;
        
        if (stateNow->ID == goalPose.row*MAX_COL + goalPose.col) {
            // Count the states in the parent chain.
            int count = 0;
            Node *cursor = stateNow;

            while (cursor != NULL) {
                count++;
                cursor = cursor->parent;
            }

            // Allocate an array of pointers to those nodes.
            Node **path = malloc(count * sizeof *path);

            if (path == NULL) {
                fprintf(stderr, "Could not allocate the path array.\n");
                return false;
            }

            // Store the route backward
            cursor = stateNow;
            int index = 0;

            while (cursor != NULL) {
                path[index++] = cursor;
                cursor = cursor->parent;
            }

            FILE *file = fopen("path_to_goal.txt", "w");

            if (file == NULL) {
                perror("Could not open puzzle_states.txt");
                free(path);
                freePriorityQueue(&frontier);
                return false;
            }

            printf("Goal %d,%d found!\n", goalPose.row , goalPose.col);

            // Read backward 
            int move = 0;

            for (int i = count - 1; i >= 0; i--) {
                // printf("Move Number %d is %d,%d \n", move, path[i]->myPose.row , path[i]->myPose.col);
                fprintf(file, "Move Number %d is %d,%d \n", move, path[i]->myPose.row , path[i]->myPose.col);
                move++;
            }

            printf("The total moves are %d\n", count - 1);
            printf("The total explored states are %d\n", exploredStates);

            fclose(file);
            free(path);
            freePriorityQueue(&frontier);

            return true;
        }
   
    }
}


int main(void)
{
    size_t total_elements = 0;
    
    // grid holds the heap memory address returned by load_occupancy_grid
    char *grid = load_occupancy_grid("occupancy_grid7.csv", &total_elements);
    printf("%d \n", grid[34*2000 + 25]);
    if (!grid) {
        fprintf(stderr, "Failed to load occupancy grid.\n");
        return EXIT_FAILURE;
    }

    printf("Successfully loaded %zu elements into heap memory.\n", total_elements);

    clock_t start_time = clock();
    

    Node *root = createNode( (Pose){1,1} , 1);
    astar(root, (Pose){997,1997} , grid ); 

    clock_t end_time = clock();
    double time_taken = (double)(end_time - start_time) / CLOCKS_PER_SEC;
    printf("The program took %f milli seconds to execute.\n", time_taken*1000.0);

    freeTree(root);

    free(grid);
    grid = NULL; 
    return 0;
    
}
    

