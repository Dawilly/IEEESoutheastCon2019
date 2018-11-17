#include <stdio.h>
#include "message.h"

typedef struct _message {
	time_t timestamp;
	char* value;
	messageType type;
} message;

char* convertMessageType(messageType);

message *newMessage(char* v, messageType type) {
	message* msg = malloc(sizeof(message));
	msg->timestamp = time(0);
	msg->value = v;
	msg->type = type;
	return msg;
}

void displayMessage(FILE* fp, void* value) {
	message* msg = value;
	char* readableTime = ctime(&msg->timestamp);
	readableTime[strlen(readableTime) - 1] = '\0';
	fprintf(fp, "[%s] [%s]: %s\n", readableTime, convertMessageType(msg->type), msg->value);
}

void freeMessage(message* msg) {
	free(msg->value);
	free(msg);
	return;
}

char* convertMessageType(messageType type) {
	switch (type) {
		case Trace:
			return "Trace";
		case Info:
			return "Info";
		case Warning:
			return "Warning";
		case Error:
			return "Error";
		case Fatal:
			return "Fatal";
	}
	return "Unknown";
}
