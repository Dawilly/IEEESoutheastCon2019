VL53 = ./VL53L0X
LOGR = ./Logger
TCAM = ./TCA9548A

OBJS = logger.o SensorManager.o VL53L0X.o iris.o TCAMux.o vl53l0x_api.o vl53l0x_platform.o vl53l0x_api_calibration.o vl53l0x_api_core.o vl53l0x_api_ranging.o vl53l0x_api_strings.o vl53l0x_setup.o
OPTS = -Wall -g -std=c++11 -pthread -lwiringPi -W
DEPS = $(VL53)/vl53l0x_setup.h $(VL53)/vl53l0x_tuning.h $(VL53)/vl53l0x_types.h $(VL53)/vl53l0x_api_calibration.h $(VL53)/vl53l0x_api_core.h $(VL53)/vl53l0x_api_ranging.h $(VL53)/vl53l0x_api_strings.h $(VL53)/vl53l0x_def.h $(VL53)/vl53l0x_device.h $(VL53)/vl53l0x_i2c_platform.h $(VL53)/vl53l0x_interrupt_threshold_settings.h $(VL53)/vl53l0x_platform_log.h

Iris : $(OBJS) 
	g++ $(OPTS) $(OBJS) -o Iris

iris.o: iris.cpp SensorManager.hpp $(VL53)/VL53L0X.hpp
	g++ $(OPTS) -c iris.cpp
	
SensorManager.o: SensorManager.cpp SensorManager.hpp $(VL53)/VL53L0X.hpp $(TCAM)/TCAMux.h
	g++ $(OPTS) -c SensorManager.cpp 
	
logger.o : $(LOGR)/logger.cpp $(LOGR)/logger.hpp $(LOGR)/message.hpp
	g++ $(OPTS) -c $(LOGR)/logger.cpp $(LOGR)/logger.hpp 

VL53L0X.o : $(VL53)/VL53L0X.cpp $(VL53)/VL53L0X.hpp $(VL53)/vl53l0x_api.h $(DEPS)
	g++ $(OPTS) -c $(VL53)/VL53L0X.cpp $(VL53)/VL53L0X.hpp

TCAMux.o : $(TCAM)/TCAMux.cpp $(TCAM)/TCAMux.h
	g++ $(OPTS) -c $(TCAM)/TCAMux.cpp $(TCAM)/TCAMux.h

#VL53L0X Dependencies

vl53l0x_api.o : $(VL53)/vl53l0x_api.c $(VL53)/vl53l0x_api.h $(DEPS)
	g++ $(OPTS) -c $(VL53)/vl53l0x_api.c $(VL53)/vl53l0x_api.h

vl53l0x_platform.o : $(VL53)/vl53l0x_platform.c $(VL53)/vl53l0x_platform.h $(DEPS)
	gcc -Wall -W -pipe -c $(VL53)/vl53l0x_platform.c $(VL53)/vl53l0x_platform.h

vl53l0x_api_calibration.o : $(VL53)/vl53l0x_api_calibration.c $(DEPS)
	g++ $(OPTS) -c $(VL53)/vl53l0x_api_calibration.c 

vl53l0x_api_core.o : $(VL53)/vl53l0x_api_core.c $(DEPS)
	gcc -Wall -W -pipe -c $(VL53)/vl53l0x_api_core.c 

vl53l0x_api_ranging.o : $(VL53)/vl53l0x_api_ranging.c $(DEPS)
	g++ $(OPTS) -c $(VL53)/vl53l0x_api_ranging.c 

vl53l0x_api_strings.o : $(VL53)/vl53l0x_api_strings.c $(DEPS)
	g++ $(OPTS) -c $(VL53)/vl53l0x_api_strings.c 

vl53l0x_setup.o : $(VL53)/vl53l0x_setup.cpp $(DEPS)
	g++ $(OPTS) -c $(VL53)/vl53l0x_setup.cpp 
	
clean :
	rm -f $(OBJS) Iris
