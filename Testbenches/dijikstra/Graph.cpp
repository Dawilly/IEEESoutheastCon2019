#include "Graph.h"
#include "Vertex.h"
#include "Edge.h"
#include <fstream>
#include <vector>

#include <iostream>

#define STREAM_SIZE 256

// Helper functions
static void makeEdge(std::ifstream *file, std::vector<Vertex *> *vertices,
        std::vector<Edge *> *edges);

// Given a formatted file of edges, create a graph and return the vertices
std::vector<Vertex *> makeGraph(std::ifstream *file) {
    std::vector<Vertex *> vertices;
    std::vector<Edge *> edges;

    // If there is a vertex pending, then an edge is pending
    while (vertexPending(file))
        makeEdge(file, &vertices, &edges);

    return vertices;
}

// Create a unique edge and add it to the list of edges. If edge is not unique,
//  then discard the edge.
// Example format: "(0.0, 0.0) : (1.0, 1.0)"
static void makeEdge(std::ifstream *file, std::vector<Vertex *> *vertices,
        std::vector<Edge *> *edges)
{
    Vertex *a = makeVertex(file, vertices);
    file->ignore(STREAM_SIZE, ':');
    Vertex *b = makeVertex(file, vertices);

    Edge *search = nullptr;
    for (std::vector<Edge *>::iterator i = edges->begin(); i != edges->end();
            i++)
    {
        // Check to see if the edge was already made
        if ((*i)->getNeighbor(a) == b) {
            search = (*i);
            break;
        }
    }
    if (search != nullptr) return;
    edges->push_back(new Edge(a, b));
    return;
}

// Create a unique vertex and add it to the list of vertices. If the vertex
//  is not unique, then return the existing vertex.
// Example format: "(0.0, 1.0)"
Vertex *makeVertex(std::ifstream *file, std::vector<Vertex *> *vertices)
{
    double x, y;
    file->ignore(STREAM_SIZE, '(');
    (*file) >> x;
    file->ignore(STREAM_SIZE, ',');
    (*file) >> y;
    file->ignore(STREAM_SIZE, ')');

    Vertex *search = nullptr;
    for (std::vector<Vertex *>::iterator i = vertices->begin();
            i != vertices->end(); i++)
    {
        // Check to see if the vertex was already made
        if ((*i)->getX() == x && (*i)->getY() == y) {
            search = (*i);
            break;
        }
    }
    if (search != nullptr) return search;
    Vertex *v = new Vertex(x, y);
    vertices->push_back(v);
    return v;
}

// Consumes all leading whitespace and checks if next character is '('. If so,
//  a vertex is pending.
bool vertexPending(std::ifstream *file) {
    (*file) >> std::ws;
    return (file->peek() == '(') ? true : false;
}
