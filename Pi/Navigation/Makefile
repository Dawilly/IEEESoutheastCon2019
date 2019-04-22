CPPFLAGS = -Wall -Wextra
GPIOFLAGS = -pthread -lpigpio
OBJ = Dijikstra.o Graph.o Edge.o Vertex.o Serial8N1.o
EXE = navigation

all: $(EXE)

$(EXE): $(OBJ)
	g++ $(CPPFLAGS) $(GPIOFLAGS) main.cpp $(OBJ) -o $(EXE)

test: $(EXE)
	sudo ./$(EXE) /dev/ttyS0 graph.txt waypoints.txt

%o: %.cpp
	g++ $(CPPFLAGS) -c $< -o $@

clean:
	rm -f $(EXE) *.o
