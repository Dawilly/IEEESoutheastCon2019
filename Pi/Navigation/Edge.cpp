#include "Edge.h"
#include "Vertex.h"
#include <algorithm>
#include <math.h>

Edge::Edge(Vertex *a, Vertex *b) {
    this->a = a;
    this->b = b;
    this->weight = sqrt(pow(a->getX() - b->getX(), 2)
            + pow(a->getY() - b->getY(), 2));
    // Add edge to each vertex adjacency list
    a->getAdjacencyList()->push_back(this);
    b->getAdjacencyList()->push_back(this);
}

Edge::~Edge() {
    // Remove the edge from each vertex adjacency list
    this->a->getAdjacencyList()->erase(
            std::remove(this->a->getAdjacencyList()->begin(),
                this->a->getAdjacencyList()->end(), this),
            this->a->getAdjacencyList()->end());
    this->b->getAdjacencyList()->erase(
            std::remove(this->b->getAdjacencyList()->begin(),
                this->b->getAdjacencyList()->end(), this),
            this->b->getAdjacencyList()->end());
}

double Edge::getWeight() {return this->weight;}

Vertex *Edge::getNeighbor(Vertex *v) {
    if (this->a == v) return this->b;
    if (this->b == v) return this->a;
    return NULL;
}
