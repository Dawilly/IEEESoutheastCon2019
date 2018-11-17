#include <stdio.h>
#include <pthread.h>
#include "logger.h"

typedef struct _logger {
	FILE* fp;
	pthread_t thread;
	pthread_mutex_t mutex;
	int threadState;
	queue* messageList;
	void (*display) (FILE *, void *);
} Logger;

void* bootstrapper(void*);

Logger* createLogger(FILE* fp) {
	Logger* newLogger = malloc(sizeof(Logger));
	newLogger->fp = fp;
	newLogger->messageList = newQueue(NULL);
	newLogger->display = displayMessage;
	return newLogger;
}

int runLogger(Logger* logger) {
	logger->threadState = pthread_create(&logger->thread, NULL, bootstrapper, (void*) logger);
	pthread_detach(logger->thread);
	return 0;
}

void* bootstrapper(void* ptr) {
	Logger* logger = (Logger*) ptr;
	int running = 1;
	while (running) {
		if (sizeQueue(logger->messageList)) {
			message* msgToPrint = dequeue(logger->messageList);
			logger->display(logger->fp, (void*)msgToPrint);
		}
	}
	return 0;
}

void addMessage(Logger* logger, char* str, messageType type) {
	message* msg = newMessage(str, type);
	enqueue(logger->messageList, msg);
	fprintf(stdout, "Added Message\n");
	return;
}