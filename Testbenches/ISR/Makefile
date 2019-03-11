CPPFLAGS = -Wall -Wextra
GPIOFLAGS = -pthread -lpigpio
EXE = isr

all:
	g++ $(CPPFLAGS) $(GPIOFLAGS) isr.cpp -o $(EXE)

test: all
	./$(EXE)

clean:
	rm -f $(EXE)
