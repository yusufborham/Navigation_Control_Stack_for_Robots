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

  
// bool astar(Node *root, Pose goalPose , char *occupicyMap) {
//     PriorityQueue frontier ;

//     if (!initPriorityQueue(&frontier, 5000))
//     return false;

//     Node *stateNow = root ;
//     int stateCosts[5000] = {0} ; 

//     stateNow->priority = computeManhattan(stateNow->myPose , goalPose) ;
//     stateCosts[(int)stateNow->ID] =  computeManhattan(stateNow->myPose , goalPose) ;

//     int exploredStates = 0 ;

//     while (1){  

        
//         // Expand The State 
//         int rowNow = stateNow->myPose.row ;
//         int colNow = stateNow->myPose.col ;

//         bool canMoveRight = (colNow != MAX_COL - 1) && !(occupicyMap[rowNow * MAX_COL + colNow + 1]) && ( ((stateNow->cost + 1) < stateCosts[rowNow * MAX_COL + colNow + 1]) || (stateCosts[rowNow * MAX_COL + colNow + 1] == 0)    )
//         // well a2dar at7rk f 4 amaken Up ,down ,left , right f ana b compute shakl el array law et7rkt f kol el amaken deh 
//         // then bat2ked en ay state gdida mat3mltsh abl keda law lesa novy ba7otha child lel parent state el ana feha delw2ty 
//         if ((zeroIndex) < 6){
//             char newState[10] = {0};
//             strcpy(newState, stateNow->state);
//             newState[zeroIndex] = stateNow->state[zeroIndex + DOWN] ;
//             newState[zeroIndex + DOWN] = '0' ;
//             int id = state_to_id(newState);
//             int newState_cost = stateNow->cost + 1 ;

//             // because at the begining all the states are initialzed b zero we7na 3arfen en mafish cost b zero
//             if ( (stateCosts[id] > newState_cost) || stateCosts[id] == 0 ){
//                 Node *child = addChild(stateNow , newState);
//                 int heur = computeManhattan(child);
//                 child->cost = newState_cost ;
//                 child->priority = heur + newState_cost ;
//                 child->ID = id ;
//                 stateCosts[id] = newState_cost ;

//                 // push the new state 
//                 if (!pushPriorityQueue(&frontier, child)) {
//                 freePriorityQueue(&frontier);
//                 return false;
//                 }
//             }    
//         }
//         if ((zeroIndex % 3) < 2 ){
//             char newState[10] = {0};
//             strcpy(newState, stateNow->state);
//             newState[zeroIndex] = stateNow->state[zeroIndex + RIGHT] ;
//             newState[zeroIndex + RIGHT] = '0' ;

//             int id = state_to_id(newState);
//             int newState_cost = stateNow->cost + 1 ;

//             if ( (stateCosts[id] > newState_cost) || stateCosts[id] == 0 ){
//                 Node *child = addChild(stateNow , newState);
//                 int heur = computeManhattan(child);
//                 child->cost = newState_cost ;
//                 child->priority = heur + newState_cost ;
//                 child->ID = id ;
//                 stateCosts[id] = newState_cost ;

//                 // push the new state 
//                 if (!pushPriorityQueue(&frontier, child)) {
//                 freePriorityQueue(&frontier);
//                 return false;
//                 }
//             }
//         }
          
//         if (zeroIndex >= 3){
//             char newState[10] = {0};
//             strcpy(newState, stateNow->state);
//             newState[zeroIndex] = stateNow->state[zeroIndex + UP] ;
//             newState[zeroIndex + UP] = '0' ;
//             int id = state_to_id(newState);
//             int newState_cost = stateNow->cost + 1 ;

//             if ( (stateCosts[id] > newState_cost) || stateCosts[id] == 0 ){
//                 Node *child = addChild(stateNow , newState);
//                 int heur = computeManhattan(child);
//                 child->cost = newState_cost ;
//                 child->priority = heur + newState_cost ;
//                 child->ID = id ;
//                 stateCosts[id] = newState_cost ;

//                 // push the new state 
//                 if (!pushPriorityQueue(&frontier, child)) {
//                 freePriorityQueue(&frontier);
//                 return false;
//                 }
//             }  
//         }

//         if ((zeroIndex % 3) > 0 ){
//             char newState[10] = {0};
//             strcpy(newState, stateNow->state);
//             newState[zeroIndex] = stateNow->state[zeroIndex + LEFT] ;
//             newState[zeroIndex + LEFT] = '0' ;

//             int id = state_to_id(newState);
//             int newState_cost = stateNow->cost + 1 ;

//             if ( (stateCosts[id] > newState_cost) || stateCosts[id] == 0 ){
//                 Node *child = addChild(stateNow , newState);
//                 int heur = computeManhattan(child);
//                 child->cost = newState_cost ;
//                 child->priority = heur + newState_cost ;
//                 child->ID = id ;
//                 stateCosts[id] = newState_cost ;

//                 // push the new state 
//                 if (!pushPriorityQueue(&frontier, child)) {
//                 freePriorityQueue(&frontier);
//                 return false;
//                 }
//             }
//         }
        
//         // check the state 
//         Node *next ;
//         while (1){
//             if (popPriorityQueue(&frontier, &next)) {
//                 // we need to check whether the poped state was popped before under a better cost so if the cost of this node is worse than the one in our 
//                 // array holding the cost of each state we shall discard it
//                 // keep looping until we pop a valid state 
//                 if( stateCosts[next->ID] >= next->cost  ) {
//                     stateNow = next ;
//                     break ;
//                 }
                
//             }
//             else {
//                 printf("Queue is empty. No more nodes to explore and could not find the Goal.\n");
//                 freePriorityQueue(&frontier);
//                 return false ;
//             }
//         }

//         // printState(stateNow->state); ---> uncomment to print the explored states 
//         exploredStates++ ;
//         if (stateCmp(stateNow->state, goal)) {
//             // Count the states in the parent chain.
//             int count = 0;
//             Node *cursor = stateNow;

//             while (cursor != NULL) {
//                 count++;
//                 cursor = cursor->parent;
//             }

//             // Allocate an array of pointers to those nodes.
//             Node **path = malloc(count * sizeof *path);

//             if (path == NULL) {
//                 fprintf(stderr, "Could not allocate the path array.\n");
//                 return false;
//             }

//             // Store the route backward
//             cursor = stateNow;
//             int index = 0;

//             while (cursor != NULL) {
//                 path[index++] = cursor;
//                 cursor = cursor->parent;
//             }

//             FILE *file = fopen("puzzle_states.txt", "w");

//             if (file == NULL) {
//                 perror("Could not open puzzle_states.txt");
//                 free(path);
//                 freePriorityQueue(&frontier);
//                 return false;
//             }

//             printf("Goal %s found!\n", goal);

//             // Read backward 
//             int move = 0;

//             for (int i = count - 1; i >= 0; i--) {
//                 printf("Move Number %d is %s\n", move, path[i]->state);
//                 fprintf(file, "Move Number %d is %s\n", move, path[i]->state);
//                 move++;
//             }

//             printf("The total moves are %d\n", count - 1);
//             printf("The total explored states are %d\n", exploredStates);

//             fclose(file);
//             free(path);
//             freePriorityQueue(&frontier);

//             return true;
//         }
   
//     }
// }


int main(void)
{
   

    // timestamp for the begining of execution to measure the running time of the program
    clock_t start_time = clock();

    // char initialState[10];

    // printf("Please enter the initial state of the puzzle (9 digits, 0-8, where 0 represents the blank space): ");
    // scanf("%9s", initialState);

    // for (int i = 0; i < 9; i++) {
    //     if (initialState[i] < '0' || initialState[i] > '8') {
    //         printf("Invalid input. Please enter exactly 9 digits from 0 to 8.\n");
    //         return 1;
    //     }
    // }

    // for (int i = 0; i < 9; i++) {
    //     for (int j = i + 1; j < 9; j++) {
    //         if (initialState[i] == initialState[j]) {
    //             printf("Invalid input. Each digit from 0 to 8 must appear exactly once.\n");
    //             return 1;
    //         }
    //     }
    // }
    
    
    // if (stateCmp(initialState , "012345678")) {
    //     printf("The Puzzle is solved already ") ;
    // }
    // else {
    //     if (is_solvable(initialState))
    //     { 
    //     Node *root = createNode(initialState , 1);
    //     puzzFAstar(root, "012345678"); 
    //     freeTree(root);
    //     }
    //     else {
    //     printf("This Puzzle Can't be solved \n ") ;
    //     }
    // }
    
    size_t total_elements = 0;
    
    // grid holds the heap memory address returned by load_occupancy_grid
    char *grid = load_occupancy_grid("occupancy_grid.csv", &total_elements);

    if (!grid) {
        fprintf(stderr, "Failed to load occupancy grid.\n");
        return EXIT_FAILURE;
    }

    printf("Successfully loaded %zu elements into heap memory.\n", total_elements);

    clock_t end_time = clock();
    double time_taken = (double)(end_time - start_time) / CLOCKS_PER_SEC;
    printf("The program took %f micro seconds to execute.\n", time_taken*1000000.0);

    // Example 1: Accessing elements directly as a 1D array
    printf("Elements from index 100 to 119:\n");
    for (size_t i = 100; i < 120 && i < total_elements; i++) {
        printf("%d ", grid[i]);
    }
    printf("\n");

    // Example 2: Accessing elements using 2D coordinates (row, col)
    // occupancy_grid.csv consists of 50 rows x 100 columns
    size_t width = 100;
    size_t row = 2;
    size_t col = 6;
    size_t index = (row * width) + col;

    if (index < total_elements) {
        printf("Value at row %zu, col %zu (index %zu): %d\n", row, col, index, grid[index]);
    }
    int stateCosts[5000] = {0} ;
    int cost = 10 ; 
    stateCosts[207] = 10 ;    
    bool canMoveRight = (col != MAX_COL - 1) && !(grid[row * MAX_COL + col + 1]) && ( ((cost + 1) < stateCosts[row * MAX_COL + col + 1]) || (stateCosts[row * MAX_COL + col + 1] == 0)    ) ;

    printf("%d \n " , canMoveRight) ;
    // Always free memory allocated on the heap when done
    free(grid);
    grid = NULL;
    return 0;
}

/* 3shan matensash el commands kal 3ada 

python3 puzzle_viewer.py puzzle_states.txt

gcc -std=c11 -Wall -Wextra astar.c TreeFoof/tree.c TreePriorityQueue/treePriorityQueue.c -o ASTAR

./ASTAR
*/
