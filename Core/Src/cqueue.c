/*
 * cqueue.c
 *
 *  Created on: Nov 13, 2020
 *      Author: Łukasz Zelek
 */

#include <cqueue.h>


void initQueue(Queue *que) {
    que->front = que->back = NULL;
    que->size = 0;
}

uint8_t empty(Queue *que) {
    return (uint8_t) ((que->size) == 0);
}

void copyToNode(uint8_t pos, Node *node) {
    node->pos = pos;
}

void copyToPosition(Node *node, uint8_t *pos) {
    *pos = node->pos;
}

void set(uint8_t pos, Queue *que) {
    Node *newNode;
    newNode = (Node *) malloc(sizeof(Node));
    copyToNode(pos, newNode);
    newNode->next = NULL;
    if (empty(que))
        que->front = newNode;
    else
        que->back->next = (struct Node *) newNode;
    que->back = newNode;
    que->size++;
}

void delete(uint8_t *pos, Queue *que) {
    Node *pointer;
    copyToPosition(que->front, pos);
    pointer = que->front;
    que->front = (Node *) que->front->next;
    free(pointer);
    que->size--;
    if (que->size == 0)
        que->back = NULL;
}
