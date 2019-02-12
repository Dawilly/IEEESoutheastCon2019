#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>

#ifndef __MESSAGE_INCLUDED__
#define __MESSAGE_INCLUDED__

typedef enum messageType {
	Trace,
	Info,
	Warning,
	Error,
	Fatal,
	None
} messageType;

typedef struct _message message;

message *newMessage(char*, messageType);   //constructor
message *newDataMessage(char*, int, int);
void displayMessageDataColl(FILE*, void*);
void displayMessage(FILE*, void*);
void freeMessage(message*);

#endif
