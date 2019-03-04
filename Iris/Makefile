OBJS = logger.o SensorManager.o VL53L0X.o I2Cdev.o iris.o
OPTS = -Wall -Wextra -g -std=c++11 -pthread -lwiringPi
VL53 = ./VL53L0X
LOGR = ./Logger

Iris : $(OBJS) 
	g++ $(OPTS) $(OBJS) -o Iris

iris.o: iris.cpp SensorManager.hpp $(VL53)/VL53L0X.hpp
	g++ $(OPTS) -c iris.cpp
	
SensorManager.o: SensorManager.cpp SensorManager.hpp $(VL53)/VL53L0X.hpp $(VL53)/I2Cdev.hpp
	g++ $(OPTS) -c SensorManager.cpp 
	
logger.o : $(LOGR)/logger.cpp $(LOGR)/logger.hpp $(LOGR)/message.hpp
	g++ $(OPTS) -c $(LOGR)/logger.cpp $(LOGR)/logger.hpp 

VL53L0X.o : $(VL53)/VL53L0X.cpp $(VL53)/VL53L0X.hpp $(VL53)/VL53L0X_defines.hpp $(VL53)/I2Cdev.hpp
	g++ $(OPTS) -c $(VL53)/VL53L0X.cpp $(VL53)/VL53L0X.hpp
	
I2Cdev.o : $(VL53)/I2Cdev.cpp $(VL53)/I2Cdev.hpp
	g++ $(OPTS) -c $(VL53)/I2Cdev.cpp $(VL53)/I2Cdev.hpp
	
clean :
	rm -f $(OBJS) Iris
