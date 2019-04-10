#include <vector>

#ifndef __VERTEX_INCLUDED__
#define __VERTEX_INCLUDED__

#include "Edge.h"
class Edge;

typedef enum {
    Red = 0,
    Green = 1,
    Blue = 2,
    Yellow = 3,
    Invalid = 4
} Color;

class Vertex {
    private:
        double x, y;
        std::vector<Edge *> adjacencyList;
        Color color;
        
        // Necessary for Dijikstra's Algorithm
        double distance;
        Vertex *previous;

    public:
        Vertex(double x, double y);
        ~Vertex();
        double getX();
        double getY();
        Color getColor();
        void setColor(Color color);
        std::vector<Edge *> *getAdjacencyList();
        
        // Necessary for Dijikstra's Algorithm
        double getDistance();
        void setDistance(double distance);
        Vertex *getPrevious();
        void setPrevious(Vertex *previous);
};

#endif
