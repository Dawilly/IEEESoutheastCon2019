#include <iostream>
#include <fstream>
#include <queue>
#include <thread>
#include "message.hpp"

#ifndef __LOGGER_CPP_INCLUDED__
#define __LOGGER_CPP_INCLUDED__

class Logger {
	private:
		std::queue<message> Messages;
		std::ofstream file;
		std::thread _thread;
		std::string delimiter;
		bool running;
		int count;
		
		static void Bootstrapper(Logger*);
		void WriteMessage(std::ofstream&, message);
	public:
		Logger(std::string, std::string);
		void AddMessage(std::string*, messageType);
		void AddMessage(int*, messageType);
		void AddMessage(double*, messageType);
		void AddMessage(void*, messageType, valueType);
		void Run();
		void Stop();
};

#endif
