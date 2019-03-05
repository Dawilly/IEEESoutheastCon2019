#include <linux/i2c-dev.h>

#ifndef __TCAMUX_INCLUDED__
#define __TCAMUX_INCLUDED__
class TCAMux {
	private:
		int status;
		int activeInput;
		char dataWrite;
	public:
		TCAMux();
		void Initialize();
		void Switch(int);
};

#endif