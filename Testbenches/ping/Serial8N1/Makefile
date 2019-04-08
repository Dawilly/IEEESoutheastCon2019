CPPFLAGS = -Wall -Wextra
GPIOFLAGS = -lpigpio -lpthread
EXE = serial

all: $(EXE)

test: all
	sudo ./$(EXE) /dev/ttyS0

$(EXE):
	g++ $(CPPFLAGS) $(GPIOFLAGS) main.cpp Serial8N1.cpp -o $(EXE)

clean:
	rm -f $(EXE)
