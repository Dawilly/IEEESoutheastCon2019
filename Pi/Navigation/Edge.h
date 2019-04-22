#ifndef __EDGE_INCLUDED__
#define __EDGE_INCLUDED__

#include "Vertex.h"
class Vertex;

class Edge {
    private:
        Vertex *a, *b;
        double weight;

    public:
        Edge(Vertex *a, Vertex *b);
        ~Edge();
        double getWeight();
        Vertex *getNeighbor(Vertex *v);
};

#endif
