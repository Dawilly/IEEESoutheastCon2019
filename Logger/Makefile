OBJS = logger.o message.o queue.o sll.o TestUnit.o
OPTS = -Wall -Wextra -g -std=c99

logger : $(OBJS) 
	gcc $(OPTS) $(OBJS) -o logger

testunit.o: TestUnit.c logger.h
	gcc $(OPTS) -c TestUnit.c
	
logger.o : logger.c logger.h queue.h sll.h message.h
	gcc $(OPTS) -c logger.c logger.h
	
queue.o : queue.c queue.h sll.h
	gcc $(OPTS) -c queue.c queue.h
	
sll.o : sll.c sll.h
	gcc $(OPTS) -c sll.c sll.h

message.o : message.c message.h
	gcc $(OPTS) -c message.c message.h

clean :
	rm -f $(OBJS) logger
