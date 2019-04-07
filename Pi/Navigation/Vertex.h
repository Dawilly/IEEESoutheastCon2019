#include <vector>

#ifndef __VERTEX_INCLUDED__
#define __VERTEX_INCLUDED__

#include "Edge.h"
class Edge;

class Vertex {
    private:
        double x, y;
        std::vector<Edge *> adjacencyList;
        
        // Necessary for Dijikstra's Algorithm
        double distance;
        Vertex *previous;

    public:
        Vertex(double x, double y);
        ~Vertex();
        double getX();
        double getY();
        std::vector<Edge *> *getAdjacencyList();
        
        // Necessary for Dijikstra's Algorithm
        double getDistance();
        void setDistance(double distance);
        Vertex *getPrevious();
        void setPrevious(Vertex *previous);
};

#endif
