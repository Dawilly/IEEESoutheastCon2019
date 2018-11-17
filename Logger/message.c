#include <stdio.h>
#include "message.h"

typedef struct _message {
	char* timestamp;
	char* value;
	messageType type;
} message;

message *newMessage(char* v, messageType type) {
	message* msg = malloc(sizeof(message));
	msg->value = v;
	msg->type = type;
	return msg;
}

void displayMessage(FILE* fp, void* value) {
	message* msg = value;
	fprintf(fp, "%s", msg->value);
}

void freeMessage(message* msg) {
	free(msg->timestamp);
	free(msg->value);
	free(msg);
	return;
}
