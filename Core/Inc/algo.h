/*
 * algo.h
 *
 *  Created on: Nov 13, 2020
 *      Author: Łukasz Zelek
 */

#ifndef INC_ALGO_H_
#define INC_ALGO_H_

#include <sched.h>

#define TRUE 1
#define FALSE 0

#define NORTH (0)
#define EAST (1)
#define SOUTH (2)
#define WEST (3)

#define MAP_SIZE_N  (16)
#define MAP_SIZE_NN (256)

#define MAPING_CLUE 1
//opis MAPING CLUE
// 1 - szuka lepszego przejazdu
// 0 - ma wyjebane, jak znajdziesz mete, wraca na start
//

typedef struct {
    uint8_t flooding[MAP_SIZE_N][MAP_SIZE_N];
    uint8_t mappedArea[MAP_SIZE_N][MAP_SIZE_N];
    uint8_t visited[MAP_SIZE_NN];
    int nextToExplore;
    int step;
    int target;
    uint8_t x;
    uint8_t y;
    uint8_t rotation;
    uint8_t actualVisitState;
    uint8_t actualSensorsState;
    int8_t mazeTypeRightHanded;
    int8_t nodeFound;
    int8_t pathReturn;
    int8_t mapIsFinished;
} microMouseState;

int query(microMouseState *state);

int goFastestWay(microMouseState *state);

void mergeSort(int arr[], int l, int r);

#endif /* INC_ALGO_H_ */
