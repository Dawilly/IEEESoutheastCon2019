#include <iostream>
#include "logger.hpp"

using namespace std;

int main(int argc, char** argv) {
	if (argc < 2) {
		cout << "Insufficient number of arguments." << endl;
		return -1;
	}
	
	bool running = true;
	Logger tehLog(string(argv[1]), " : ");
	string* v;
	string in;
	
	tehLog.Run();
	
	while(running) {
		
		cin >> in;
		v = new string(in);
		if (in == "STOP") {
			tehLog.Stop();
			running = false;
		} else {
			tehLog.AddMessage(v, None);
		}
	}
	
	return 0;
}