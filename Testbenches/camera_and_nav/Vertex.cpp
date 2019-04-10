#include "Vertex.h"
#include "Edge.h"
#include <vector>

Vertex::Vertex(double x, double y) : adjacencyList(){
    this->x = x;
    this->y = y;
    this->Color = Invalid;
    this->distance = -1.0;
    this->previous = nullptr;
}

Vertex::~Vertex() {}

double Vertex::getX() {return this->x;}

double Vertex::getY() {return this->y;}

std::vector<Edge *> *Vertex::getAdjacencyList() {return &(this->adjacencyList);}

Color Vertex::getColor() {return this->Color;}

void Vertex::setColor(Color color) {this->Color = color;}

// Necessary for Dijikstra's Algorithm
double Vertex::getDistance() {return this->distance;}

void Vertex::setDistance(double distance) {this->distance = distance;}

Vertex *Vertex::getPrevious() {return this->previous;}

void Vertex::setPrevious(Vertex *previous) {this->previous = previous;}
