CPPFLAGS = -Wall -Wextra
EXE = serial

all: $(EXE)

test: all
	./$(EXE) /dev/cu.usbmodem14101

$(EXE):
	g++ $(CPPFLAGS) main.cpp Serial8N1.cpp -o $(EXE)

clean:
	rm -f $(EXE)
