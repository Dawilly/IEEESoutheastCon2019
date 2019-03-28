#include <stdio.h>

//David Weil - CS201-001 - Spring 17
//sll.c - Single Link List Library
//Last revision: 1/25/2017

#ifndef __SLL_INCLUDED__
#define __SLL_INCLUDED__

typedef struct SLL sll;

sll *newSLL(void(*d)(FILE *, void *));            //constructor
void insertSLL(sll *items, int index, void *value); //stores a generic value
void *removeSLL(sll *items, int index);            //returns a generic value
void unionSLL(sll *recipient, sll *donor);         //merge two lists into one
int sizeSLL(sll *items);
void displaySLL(FILE *, sll *items);
void *getSLL(sll *items, int index);				 //Do we need this?

#endif