OBJS = logger.o message.o queue.o sll.o sensor.o
OPTS = -Wall -Wextra -g -std=c11 -lpigpio -lpthread

sonar : $(OBJS) 
	gcc $(OPTS) $(OBJS) -o sonar

sensor.o: sensor.c logger.h
	gcc $(OPTS) -c sensor.c 
	
logger.o : logger.c logger.h queue.h sll.h message.h
	gcc $(OPTS) -pthread -c logger.c logger.h
	
queue.o : queue.c queue.h sll.h
	gcc $(OPTS) -c queue.c queue.h
	
sll.o : sll.c sll.h
	gcc $(OPTS) -c sll.c sll.h

message.o : message.c message.h
	gcc $(OPTS) -c message.c message.h

clean :
	rm -f $(OBJS) sonar

