/*
 * cqueue.h
 *
 *  Created on: Nov 13, 2020
 *      Author: Łukasz Zelek
 */

#ifndef INC_CQUEUE_H_
#define INC_CQUEUE_H_

#include <sched.h>
#include <stdlib.h>

typedef struct {
    uint8_t pos;
    struct Node *next;
} Node;

typedef struct {
    Node *front;
    Node *back;
    uint8_t size;
} Queue;

void initQueue(Queue *que);
uint8_t empty(Queue *que);
void copyToNode(uint8_t pos, Node *node);
void copyToPosition(Node *node, uint8_t *pos);
void set(uint8_t pos, Queue *que);
void delete(uint8_t *pos, Queue *que);

#endif /* INC_CQUEUE_H_ */
