OBJS = logger.o VL53Tester.o VL53L0X.o I2Cdev.o
OPTS = -Wall -Wextra -g -std=c++11 -pthread

irtester : $(OBJS) 
	g++ $(OPTS) $(OBJS) -o irtester

VL53Tester.o: VL53Tester.cpp logger.hpp VL53L0X.hpp I2Cdev.hpp
	g++ $(OPTS) -c VL53Tester.cpp 
	
logger.o : logger.cpp logger.hpp message.hpp
	g++ $(OPTS) -c logger.cpp logger.hpp 

VL53L0X.o : VL53L0X.cpp VL53L0X.hpp VL53L0X_defines.hpp I2Cdev.hpp
	g++ $(OPTS) -c VL53L0X.cpp VL53L0X.hpp
	
I2Cdev.o : I2Cdev.cpp I2Cdev.hpp
	g++ $(OPTS) -c I2Cdev.cpp I2Cdev.hpp
	
clean :
	rm -f $(OBJS) irtester
