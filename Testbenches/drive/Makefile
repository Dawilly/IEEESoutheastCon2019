CPPFLAGS = -Wall -Wextra
GPIOFLAGS = -pthread -lpigpio
EXE = serial

all:
	g++ $(CPPFLAGS) $(GPIOFLAGS) main.cpp Serial8N1.cpp -o $(EXE)

test: all
	./$(EXE) /dev/ttyS0

clean:
	rm -f $(EXE)
