#include <iostream>

#ifndef __MESSAGE_CPP_INCLUDED__
#define __MESSAGE_CPP_INCLUDED__

enum messageType {
	Trace,
	Info,
	Warning,
	Error,
	Fatal,
	None
};

typedef struct _message {
	time_t timestamp;
	string value;
	messageType type;
} message;

#endif