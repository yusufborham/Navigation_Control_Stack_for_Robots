#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>


#include "./TreeFoof/tree.h"
#include "./TreePriorityQueue/treePriorityQueue.h"


/**
 * Loads a CSV file containing grid values into a dynamically allocated 1D char array.
 * Values equal to 100 are mapped to 1; all other values are mapped to 0.
 *
 * @param filename  Path to the CSV file.
 * @param out_size  Pointer to store the total number of elements read.
 * @return          Pointer to the 1D char array on the heap (or NULL on failure).
 */
char* load_occupancy_grid(const char *filename, size_t *out_size) {
    FILE *fp = fopen(filename, "r");
    if (!fp) {
        perror("Error opening file");
        return NULL;
    }

    size_t capacity = 1024;
    size_t count = 0;
    char *grid = (char *)malloc(capacity * sizeof(char));
    if (!grid) {
        fclose(fp);
        return NULL;
    }

    // Buffer sized to hold long CSV rows (e.g., 100+ columns)
    char buffer[4096];

    while (fgets(buffer, sizeof(buffer), fp)) {
        // Skip comment lines (e.g., "# start,0,0") and blank lines
        if (buffer[0] == '#' || buffer[0] == '\n' || buffer[0] == '\r') {
            continue;
        }

        // Tokenize line by comma, space, and newline delimiters
        char *token = strtok(buffer, ",\r\n ");
        while (token != NULL) {
            int val = atoi(token);
            char mapped_val = (val == 100) ? 1 : 0;

            // Expand dynamic array when capacity is reached
            if (count >= capacity) {
                capacity *= 2;
                char *temp = (char *)realloc(grid, capacity * sizeof(char));
                if (!temp) {
                    free(grid);
                    fclose(fp);
                    return NULL;
                }
                grid = temp;
            }

            grid[count++] = mapped_val;
            token = strtok(NULL, ",\r\n ");
        }
    }

    fclose(fp);

    // Optional: shrink allocated block to the exact number of elements read
    if (count > 0 && count < capacity) {
        char *exact_grid = (char *)realloc(grid, count * sizeof(char));
        if (exact_grid) {
            grid = exact_grid;
        }
    }

    *out_size = count;
    return grid;
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

    if (!initPriorityQueue(&frontier, 5000))
    return false;

    Node *stateNow = root ;
    int stateCosts[5000] = {0} ; 

    stateNow->priority = computeManhattan(stateNow->myPose , goalPose) ;
    stateCosts[(int)stateNow->ID] =  computeManhattan(stateNow->myPose , goalPose) ;

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
            neighbour->priority = computeManhattan(neighbour->myPose ,goalPose);
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
            neighbour->priority = computeManhattan(neighbour->myPose ,goalPose);
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
            neighbour->priority = computeManhattan(neighbour->myPose ,goalPose);
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
            neighbour->priority = computeManhattan(neighbour->myPose ,goalPose);
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
                printf("Move Number %d is %d,%d \n", move, path[i]->myPose.row , path[i]->myPose.col);
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
   

    // timestamp for the begining of execution to measure the running time of the program
    clock_t start_time = clock();
    size_t total_elements = 0;
    
    // grid holds the heap memory address returned by load_occupancy_grid
    char *grid = load_occupancy_grid("occupancy_grid.csv", &total_elements);

    if (!grid) {
        fprintf(stderr, "Failed to load occupancy grid.\n");
        return EXIT_FAILURE;
    }

    printf("Successfully loaded %zu elements into heap memory.\n", total_elements);

    
    Node *root = createNode( (Pose){1,1} , 1);
    astar(root, (Pose){47,1} , grid ); 

    clock_t end_time = clock();
    double time_taken = (double)(end_time - start_time) / CLOCKS_PER_SEC;
    printf("The program took %f micro seconds to execute.\n", time_taken*1000000.0);

    freeTree(root);

    free(grid);
    grid = NULL; 
    return 0;
    
}
    

