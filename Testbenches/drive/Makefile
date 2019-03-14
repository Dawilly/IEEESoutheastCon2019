CPPFLAGS = -Wall -Wextra
EXE = serial

all:
	g++ $(CPPFLAGS) main.cpp Serial8N1.cpp -o $(EXE)

test: all
	./$(EXE) /dev/ttyS0

clean:
	rm -f $(EXE)
