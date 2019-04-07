#include "Vertex.h"
#include "Edge.h"
#include <deque>
#include <vector>

#ifndef __DIJIKSTRA_INCLUDED__
#define __DIJIKSTRA_INCLUDED__

extern void Dijikstra(std::vector<Vertex *> Q, Vertex *source);
extern std::deque<Vertex *> shortestPath(Vertex *source, Vertex *target);

#endif
