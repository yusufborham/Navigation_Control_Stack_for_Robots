#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>


#include "./TreeFoof/tree.h"
#include "./TreePriorityQueue/treePriorityQueue.h"

enum {
    UP = -3 ,
    LEFT = -1 ,
    RIGHT = 1 ,
    DOWN = 3
} POSITIONS_t ;

const int FACTORIALS[] = {1, 1, 2, 6, 24, 120, 720, 5040, 40320}; // 3shan 2ady unique ID l kol e7temal

// 3shan a compare 3add 2 states 
bool stateCmp(char* state , char* goal){
    for (int i = 0 ; i < 9 ; i++){
        if (state[i] != goal[i])
            return false ;
    }
    return true ;
}


// kont nasy eny mmkn a print string b %s bas 5las b2a katbt el function lol
void printState(char* state){
   printf("%s\n",state);
}


int findZeroInState(char* state){
    for (int i = 0 ; i < 9 ; i++){
        if ( state[i] == '0' )
        return i ;
    }
    return 10 ;
}

int xGoals[] = {0 , 0 , 0 , 1 , 1 , 1 , 2 , 2, 2 };
int yGoals[] = {0 , 1 , 2 , 0 , 1 , 2 , 0 , 1 , 2};

int computeManhattan(Node* thisState){
    int heur = 0;
    for (int i = 0 ; i < 9 ; i++){
        int value = thisState->state[i] - '0' ;
        if ( value == 0 )    
            continue;       // we shall execlude el blank (zero) 3shan yb2a admissable 
        int xCurr = i / 3 ;
        int yCurr = i % 3 ;
        heur+= ( abs(xCurr - xGoals[value]) +abs(yCurr - yGoals[value]) ) ;
    }
    return heur ;
}

// b convert kol state l unique ID 
int state_to_id(char* state) {
    int id = 0;
    
    for (int i = 0; i < 9; i++) {
        int count = 0;
        
        // kam rakm as8r mn [i] we 3la his right el mfrood en ay rakm yb2a kol el as8ar mno 3la shemalo fa a higher value means 
        // worst solution aw bat7rk f etegah 8lat mmkn ast5dha m3 el A star 3mtn we ashoof hyb2a a7san mn el manhatan wla eh 
        for (int j = i + 1; j < 9; j++) {
            if (state[j] < state[i]) {
                count++;
            }
        }
        
        // badrb kol factorial fel remaianing positions 
        id += count * FACTORIALS[8 - i];
    }
    
    return id;
}


bool is_solvable(const char *state)
{
    if (state == NULL || strlen(state) != 9)
        return false;

    // Validate: every digit from 0 through 8 appears exactly once.
    bool seen[9] = {false};

    for (int i = 0; i < 9; i++) {
        if (state[i] < '0' || state[i] > '8')
            return false;

        int tile = state[i] - '0';

        if (seen[tile])
            return false;

        seen[tile] = true;
    }

    // Count inversions, excluding the blank.
    int inversions = 0;

    for (int i = 0; i < 9; i++) {
        if (state[i] == '0')
            continue;

        for (int j = i + 1; j < 9; j++) {
            if (state[j] == '0')
                continue;

            if (state[i] > state[j])
                inversions++;
        }
    }

    return inversions % 2 == 0;
}

  
bool puzzFAstar(Node *root, char* goal) {
    PriorityQueue frontier ;

    if (!initPriorityQueue(&frontier, 362880))
    return false;

    Node *stateNow = root ;
    int stateCosts[362880] = {0} ; // 9! = 362880

    // tyb delw2ty ana mmkn awsl l state bas i discovered it before f bel ID beta3ha h check el cost beta3 el wesol leha bel tare2a el adima we akarenha bel 
    // new one law la2tha aw7ash h discard it law a7sn f ha7otha fel queue 3ady bas hyb2a feh 2 nodes fel queue by coresspondo l nafs el state 
    // f ana h keep tracking of it ezay lama h push state gowa el queue h update el cost beta3ha f law get 3mlt pop l state we la2et el cost beta3ha akbr mn 
    // el fel queue f m3naha en ana 3amlt pop leha abl keda b cost a2al f ha3mlha discard 

    stateNow->priority = computeManhattan(stateNow) ;
    stateCosts[state_to_id(stateNow->state)] =  computeManhattan(stateNow);

    int exploredStates = 0 ;

    while (1){  

        int zeroIndex = findZeroInState(stateNow->state);

        // Expand The State 

        // well a2dar at7rk f 4 amaken Up ,down ,left , right f ana b compute shakl el array law et7rkt f kol el amaken deh 
        // then bat2ked en ay state gdida mat3mltsh abl keda law lesa novy ba7otha child lel parent state el ana feha delw2ty 
        if ((zeroIndex) < 6){
            char newState[10] = {0};
            strcpy(newState, stateNow->state);
            newState[zeroIndex] = stateNow->state[zeroIndex + DOWN] ;
            newState[zeroIndex + DOWN] = '0' ;
            int id = state_to_id(newState);
            int newState_cost = stateNow->cost + 1 ;

            // because at the begining all the states are initialzed b zero we7na 3arfen en mafish cost b zero
            if ( (stateCosts[id] > newState_cost) || stateCosts[id] == 0 ){
                Node *child = addChild(stateNow , newState);
                int heur = computeManhattan(child);
                child->cost = newState_cost ;
                child->priority = heur + newState_cost ;
                child->ID = id ;
                stateCosts[id] = newState_cost ;

                // push the new state 
                if (!pushPriorityQueue(&frontier, child)) {
                freePriorityQueue(&frontier);
                return false;
                }
            }    
        }
        if ((zeroIndex % 3) < 2 ){
            char newState[10] = {0};
            strcpy(newState, stateNow->state);
            newState[zeroIndex] = stateNow->state[zeroIndex + RIGHT] ;
            newState[zeroIndex + RIGHT] = '0' ;

            int id = state_to_id(newState);
            int newState_cost = stateNow->cost + 1 ;

            if ( (stateCosts[id] > newState_cost) || stateCosts[id] == 0 ){
                Node *child = addChild(stateNow , newState);
                int heur = computeManhattan(child);
                child->cost = newState_cost ;
                child->priority = heur + newState_cost ;
                child->ID = id ;
                stateCosts[id] = newState_cost ;

                // push the new state 
                if (!pushPriorityQueue(&frontier, child)) {
                freePriorityQueue(&frontier);
                return false;
                }
            }
        }
          
        if (zeroIndex >= 3){
            char newState[10] = {0};
            strcpy(newState, stateNow->state);
            newState[zeroIndex] = stateNow->state[zeroIndex + UP] ;
            newState[zeroIndex + UP] = '0' ;
            int id = state_to_id(newState);
            int newState_cost = stateNow->cost + 1 ;

            if ( (stateCosts[id] > newState_cost) || stateCosts[id] == 0 ){
                Node *child = addChild(stateNow , newState);
                int heur = computeManhattan(child);
                child->cost = newState_cost ;
                child->priority = heur + newState_cost ;
                child->ID = id ;
                stateCosts[id] = newState_cost ;

                // push the new state 
                if (!pushPriorityQueue(&frontier, child)) {
                freePriorityQueue(&frontier);
                return false;
                }
            }  
        }

        if ((zeroIndex % 3) > 0 ){
            char newState[10] = {0};
            strcpy(newState, stateNow->state);
            newState[zeroIndex] = stateNow->state[zeroIndex + LEFT] ;
            newState[zeroIndex + LEFT] = '0' ;

            int id = state_to_id(newState);
            int newState_cost = stateNow->cost + 1 ;

            if ( (stateCosts[id] > newState_cost) || stateCosts[id] == 0 ){
                Node *child = addChild(stateNow , newState);
                int heur = computeManhattan(child);
                child->cost = newState_cost ;
                child->priority = heur + newState_cost ;
                child->ID = id ;
                stateCosts[id] = newState_cost ;

                // push the new state 
                if (!pushPriorityQueue(&frontier, child)) {
                freePriorityQueue(&frontier);
                return false;
                }
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
        if (stateCmp(stateNow->state, goal)) {
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

            FILE *file = fopen("puzzle_states.txt", "w");

            if (file == NULL) {
                perror("Could not open puzzle_states.txt");
                free(path);
                freePriorityQueue(&frontier);
                return false;
            }

            printf("Goal %s found!\n", goal);

            // Read backward 
            int move = 0;

            for (int i = count - 1; i >= 0; i--) {
                printf("Move Number %d is %s\n", move, path[i]->state);
                fprintf(file, "Move Number %d is %s\n", move, path[i]->state);
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

    char initialState[10];

    printf("Please enter the initial state of the puzzle (9 digits, 0-8, where 0 represents the blank space): ");
    scanf("%9s", initialState);

    for (int i = 0; i < 9; i++) {
        if (initialState[i] < '0' || initialState[i] > '8') {
            printf("Invalid input. Please enter exactly 9 digits from 0 to 8.\n");
            return 1;
        }
    }

    for (int i = 0; i < 9; i++) {
        for (int j = i + 1; j < 9; j++) {
            if (initialState[i] == initialState[j]) {
                printf("Invalid input. Each digit from 0 to 8 must appear exactly once.\n");
                return 1;
            }
        }
    }
    
    
    if (stateCmp(initialState , "012345678")) {
        printf("The Puzzle is solved already ") ;
    }
    else {
        if (is_solvable(initialState))
        { 
        Node *root = createNode(initialState , 1);
        puzzFAstar(root, "012345678"); 
        freeTree(root);
        }
        else {
        printf("This Puzzle Can't be solved \n ") ;
        }
    }
    clock_t end_time = clock();
    double time_taken = (double)(end_time - start_time) / CLOCKS_PER_SEC;
    printf("The program took %f seconds to execute.\n", time_taken);

    return 0;
}

/* 3shan matensash el commands kal 3ada 

python3 puzzle_viewer.py puzzle_states.txt

gcc -std=c11 -Wall -Wextra astar.c TreeFoof/tree.c TreePriorityQueue/treePriorityQueue.c -o ASTAR

./ASTAR
*/
