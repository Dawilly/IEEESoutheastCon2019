#include "Dijikstra.h"
#include "Vertex.h"
#include "Edge.h"
#include <algorithm>
#include <deque>
#include <vector>

struct compareVertex {
    bool operator() (Vertex *lhs, Vertex *rhs) {
        if (lhs->getDistance() == -1.0 && rhs->getDistance() == -1.0)
            return false;
        if (lhs->getDistance() == -1.0)
            return true;
        if (rhs->getDistance() == -1.0)
            return false;
        return lhs->getDistance() > rhs->getDistance();
    }
};

void Dijikstra(std::vector<Vertex *> Q, Vertex *source) {
    source->setDistance(0.0);

    for (std::vector<Vertex *>::iterator i = Q.begin(); i != Q.end(); i++) {
        if ((*i) != source) (*i)->setDistance(-1.0);
        (*i)->setPrevious(nullptr);
    }
    
    make_heap(Q.begin(), Q.end(), compareVertex());

    while (Q.size() > 0) {
        pop_heap(Q.begin(), Q.end(), compareVertex());
        Vertex *u = Q.back();
        Q.pop_back();
        for (std::vector<Edge *>::iterator i = u->getAdjacencyList()->begin();
                i != u->getAdjacencyList()->end(); i++)
        {
            Vertex *v = (*i)->getNeighbor(u);
            double alt = u->getDistance() + (*i)->getWeight();
            if (v->getDistance() == -1.0 || alt < v->getDistance()) {
                v->setDistance(alt);
                v->setPrevious(u);
                make_heap(Q.begin(), Q.end(), compareVertex());
            }
        }
    }
}

std::deque<Vertex *> shortestPath(Vertex *source, Vertex *target) {
    std::deque<Vertex *> path;
    Vertex *u = target;
    if (u->getPrevious() != nullptr || u == source) {
        while (u != nullptr) {
            path.push_front(u);
            u = u->getPrevious();
        }
    }
    return path;
}
