#include <iostream>
#include <vector>
#include <thread>
#include "message.hpp"

#ifndef __LOGGER_CPP_INCLUDED__
#define __LOGGER_CPP_INCLUDED__

using namespace std;

class Logger {
	private:
		vector<message> Messages;
		ifstream &file;
		
		Bootstrapper();
		WriteMessage(ifstream&, message);
	public:
		Logger(string);
		void AddMessage(string v, messageType type);
		void Run();
}

#endif
