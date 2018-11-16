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

void bootstrapper(Logger*);

Logger* createLogger(FILE* fp) {
	Logger* newLogger = malloc(sizeof(Logger));
	newLogger->fp = fp;
	newLogger->messageList = newQueue(displayMessage);
}

int runLogger(Logger* logger) {
	logger->threadState = pthread_create(&logger->thread, NULL, bootstrapper, logger);
	pthread_detach(logger->thread);
	return 0;
}

void bootstrapper(Logger* logger) {
	int running = 1;
	while (running) {
		if (sizeQueue(logger->messageList)) {
			message* msgToPrint = dequeue(logger->messageList);
			logger->display(msgToPrint, logger->fp);
		}
	}
}

void addMsg(Logger* logger, char* str, messageType type) {
	message* msg = newMessage(str, type);
	enqueue(logger->messageList, msg);
	return;
}