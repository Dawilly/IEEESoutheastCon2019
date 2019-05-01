#include <iostream>
#include <algorithm>
#include <deque>
#include <time.h>
#include <vector>
#include "Dijikstra.h"
#include "Graph.h"
#include "Edge.h"
#include "Vertex.h"

using namespace std;

int main(int argc, char **argv) {
    // Check number of arguments
    if (argc < 3) {
        cerr << "Too few arguments!" << endl;
        return -1;
    }

    // Initialize the start time and timeout flag
    time_t start_time = time(0);
    bool timeout = false;

    // Initialize a graph given a file of edges
    ifstream graph_file(argv[1], ifstream::in);
    vector<Vertex *> graph = makeGraph(&graph_file);
    graph_file.close();

    // Create a set of waypoints
    ifstream waypoints_file(argv[2], ifstream::in);
    vector<Vertex *> waypoints;
    while (vertexPending(&waypoints_file))
        waypoints.push_back(makeVertex(&waypoints_file, &graph));
    waypoints_file.close();

    // Identify the four corners of the playing field
    vector<Vertex *> corners(4, nullptr);
    for (vector<Vertex *>::iterator c = graph.begin(); c != graph.end(); c++) {
        if ((*c)->getX() == 13.75 && (*c)->getY() == 13.75) {
            corners[0] = (*c);
        }
        else if ((*c)->getX() == 13.75 && (*c)->getY() == 83.25) {
            corners[1] = (*c);
        }
        else if ((*c)->getX() == 83.25 && (*c)->getY() == 13.75) {
            corners[2] = (*c);
        }
        else if ((*c)->getX() == 83.25 && (*c)->getY() == 83.25) {
            corners[3] = (*c);
        }
    }
    if (find(corners.begin(), corners.end(), nullptr) != corners.end()) {
        cerr << "Unable to find all four corners." << endl;
        return -1;
    }

    // Find and display the shortest path to each point
    Vertex *position = waypoints[0];
    for (vector<Vertex *>::iterator w = waypoints.begin() + 1;
            w != waypoints.end(); w++)
    {
        // Handle irregular operation
        Vertex *target = (*w);
        if (!timeout) {
            string response;
            cout << "Go to a colored corner (Y/n)? ";
            cin >> response;
            if (response == "y" || response == "Y") {
                int index;
                cout << "Which corner (0-3)? ";
                cin >> index;
                target = corners[index];
            }
        }

        // Use Dijikstra's algorithm to determin the shortest path
        Dijikstra(graph, position);
        deque<Vertex *> path = shortestPath(position, target);
        path.pop_front(); // First vertex is position, so ignore it

        // Follow the shortest path
        Vertex *end = nullptr;
        cout << "(" << position->getX() << ", " << position->getY() << ")";
        for (; !path.empty(); path.pop_front()) {
            // Get the target vertex
            end = path.front();

            // Print the path and travel
            cout << " -> (" << end->getX() << ", " << end->getY() << ")";

            // Update position
            position = end;
        }
        cout << endl;

        // Handle irregular operation
        if (target != (*w)) {
            w--;
        }

        // Determine if we are running out of time and need to go home
        time_t current_time = time(0);
        if (!timeout && difftime(current_time, start_time) >= 140.0) {
            // Set the waypoint itertor to our last waypoint for the next
            //  iteration
            w = waypoints.end() - 2;

            // Update the start time to see how long it takes us to get home
            start_time = time(0);

            // Timeout occured
            timeout = true;
        }
    }

    // Inidicate the time of run in seconds
    time_t current_time = time(0);
    if (timeout) {
        cout << "After timeout, we got home in "
             << difftime(current_time, start_time) << " seconds." << endl;
    }
    else {
        cout << "Total run was " << difftime(current_time, start_time)
             << " seconds." << endl;
    }

    return 0;
}
