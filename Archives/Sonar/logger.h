#include <stdio.h>
#include <stdlib.h>
#include "queue.h"
#include "message.h"

#ifndef __LOGGER_INCLUDED__
#define __LOGGER_INCLUDED__

typedef struct _logger Logger;

Logger* createLogger(FILE*, int);
int runLogger(Logger*);
void addMessage(Logger*, char*, messageType);
void addDataMessage(Logger*, int, int, char*);

#endif
