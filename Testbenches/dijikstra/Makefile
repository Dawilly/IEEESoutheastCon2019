CPPFLAGS = -Wall -Wextra
OBJ = Dijikstra.o Graph.o Edge.o Vertex.o
EXE = dijikstra

all: $(EXE)

$(EXE): $(OBJ)
	g++ $(OBJ) main.cpp -o $(EXE)

test: $(EXE)
	./$(EXE) graph.txt waypoints.txt

%.o: %.cpp
	g++ $(CPPFLAGS) -c $< -o $@

clean:
	rm -f $(EXE) *.o
