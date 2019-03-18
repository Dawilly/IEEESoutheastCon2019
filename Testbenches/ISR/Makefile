CPPFLAGS = -Wall -Wextra
GPIOFLAGS = -pthread -lpigpio
EXE = isr

all: $(EXE)

test: all
	sudo ./$(EXE)

$(EXE):
	g++ $(CPPFLAGS) $(GPIOFLAGS) isr.cpp -o $(EXE)

clean:
	rm -f $(EXE)
