#include <iostream>
#include <string>
#include <time.h>

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

enum valueType {
	Integer,
	Real,
	Str
};

typedef struct _message {
	time_t timestamp;
	void* value;
	messageType mType;
	valueType vType;
} message;

#endif
