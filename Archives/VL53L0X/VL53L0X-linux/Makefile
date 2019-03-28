OBJS = logger.o main.o VL53L0X.o I2Cdev.o
OPTS = -Wall -Wextra -g -std=c++11 -pthread

Iris : $(OBJS) 
	g++ $(OPTS) $(OBJS) -o Iris

main.o: main.cpp VL53L0X.hpp I2Cdev.hpp
	g++ $(OPTS) -c main.cpp 

VL53L0X.o : VL53L0X.cpp VL53L0X.hpp VL53L0X_defines.hpp I2Cdev.hpp
	g++ $(OPTS) -c VL53L0X.cpp VL53L0X.hpp
	
I2Cdev.o : I2Cdev.cpp I2Cdev.hpp
	g++ $(OPTS) -c I2Cdev.cpp I2Cdev.hpp
	
clean :
	rm -f $(OBJS) Iris
