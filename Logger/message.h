#include <stdio.h>

#ifndef __MESSAGE_INCLUDED__
#define __MESSAGE_INCLUDED__

typedef enum messageType {
	Trace,
	Info,
	Warning,
	Error,
	Fatal
} messageType;

typedef struct _message message;

message *newMessage(char*, messageType);   //constructor
void displayMessage(FILE*, void*);
void freeMessage(message*);

#endif
