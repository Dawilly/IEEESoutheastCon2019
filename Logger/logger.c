#include <stdio.h>
#include <pthread.h>

typedef struct _logger {
	FILE* fp;
	pthread_t thread;
	int threadState;
} Logger;

Logger* createLogger(FILE* fp) {
	Logger* newLogger = malloc(sizeof(Logger));
	newLogger->fp = fp;
}

int runLogger(Logger* logger) {
	logger->threadState = pthread_create(&logger->thread, NULL, bootstrapper, logger);
	return 0;
}

void* bootstrapper(Logger* l) {
	int running = 1;
	
	while (running) {

	}

}