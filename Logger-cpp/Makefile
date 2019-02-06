OBJS = logger.o testunit.o
OPTS = -Wall -Wextra -g -std=c++11 -pthread

logger : $(OBJS) 
	g++ $(OPTS) $(OBJS) -o logger

testunit.o: testunit.cpp logger.hpp
	g++ $(OPTS) -c testunit.cpp 
	
logger.o : logger.cpp logger.hpp message.hpp
	g++ $(OPTS) -c logger.cpp logger.hpp 

clean :
	rm -f $(OBJS) logger
