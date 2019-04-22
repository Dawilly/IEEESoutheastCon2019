#include "Vertex.h"
#include "Edge.h"
#include <fstream>
#include <vector>

#ifndef __GRAPH_INCLUDED__
#define __GRAPH_INCLUDED__

extern std::vector<Vertex *> makeGraph(std::ifstream *file);
extern Vertex *makeVertex(std::ifstream *file,
        std::vector<Vertex *> *vertices);
extern bool vertexPending(std::ifstream *file);

#endif
