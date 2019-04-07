#include "Dijikstra.h"
#include "Graph.h"
#include "Edge.h"
#include "Vertex.h"
#include "Serial8N1.h"
#include <iostream>
#include <deque>
#include <string>
#include <thread>
#include <vector>
#include <math.h>
#include <pigpio.h>

#define PIN 18
inline double readingsToInches(double r1, double r2) {
    return ((r1 + r2) / 25.4 / 2.0) + 4.5;
}
#define PI 3.14159265358979323846
#define radiansToDegrees(theta) (theta * 180.0 / PI)

using namespace std;

vector<double> correct(Vertex *v, double heading, vector<double> readings);
string makeTurnCommand(vector<double> point, Vertex *end, double *heading);
string makeDriveCommand(vector<double> point, Vertex *end, double heading);
void readArduinoData(Serial8N1 *arduino, double *heading,
        vector<double> *readings);
void handle_interrupt(int gpio, int level, uint32_t tick, void *flag);

bool arduino_ready = true;

int main(int argc, char **argv) {
    // Check number of arguments
    if (argc < 4) {
        cerr << "Too few arguments!" << endl;
        return -1;
    }

    // Set up GPIO 18 as "interrupt" to indicate arduino is ready for a message
    gpioInitialise();
    gpioSetMode(PIN, PI_INPUT);
    gpioSetPullUpDown(PIN, PI_PUD_UP);

    // Pass function to handle interrupt at rising and falling edges
    gpioSetISRFuncEx(PIN, EITHER_EDGE, 0, handle_interrupt, &arduino_ready);
    
    // Open serial port and use IRs to be parallel with left wall
    Serial8N1 arduino(argv[1], 9600);
    
    // Create a graph of the playing field
    ifstream graph_file(argv[2], ifstream::in);
    vector<Vertex *> graph = makeGraph(&graph_file);
    graph_file.close();

    // Create a set of waypoints
    ifstream waypoints_file(argv[3], ifstream::in);
    vector<Vertex *> waypoints;
    while (vertexPending(&waypoints_file))
        waypoints.push_back(makeVertex(&waypoints_file, &graph);
    waypoints_file.close();
    
    // Initialize IMU heading and IR readings
    double heading;
    vector<double> readings(8, 0.0);
    readArduinoData(&arduino, &heading, &readings);
    
    // Initialize position and begin iterating through waypoints
    Vertex *position = waypoints[0];
    for (vector<Vertex *>::iterator w = waypoints.begin() + 1;
            w != waypoints.end(); w++)
    {
        // TO-DO: Handle irregular operation
        
        // Use Dijikstra's algorithm to determine shortest path
        Dijikstra(graph, position);
        deque<Vertex *> path = shortestPath(position, (*w));
        path.pop_front(); // First vertex is position, so ignore it

        // Follow the shortest path by executing commands
        Vertex *start = position, *end = NULL;
        for (; !path.empty(); path.pop_front()) {
            // Get the target vertex
            end = path.front();
            
            // Calculate and perform a turn command
            vector<double> point = correct(start, heading, readings);
            string command = makeTurnCommand(point, end, &heading);
            arduino.write(command);
            arduino_ready = false;
            while(arduino_ready != true);
            
            // Calculate and perform a drive command
            command = makeDriveCommand(point, end, heading);
            arduino.write(command);
            arduino_ready = false;
            while(arduino_ready != true);
        
            // Update IMU heading, IR readings, and start
            readArduinoData(&arduino, &heading, &readings);
            start = end;
        }
        
        // Update the position
        position = (*w);
    }


    // Join GPIO thread and finish
    gpioTerminate();
    return 0;
}

vector<double> correct(Vertex *v, double heading, vector<double> readings) {
    double x, y;
    // Direction is up
    if ((heading > 355.0 && heading < 360.0) ||
            (heading > 0.0 && heading < 5.0))
    {
        x = (v->getX() > 48.5) ?
            (97.0 - readingsToInches(readings[4], readings[5]))
            : (readingsToInches(readings[0], readings[1]));
        y = (v->getY() > 48.5) ?
            (97.0 - readingsToInches(readings[2], readings[3]))
            : (readingsToInches(readings[6], readings[7]));
    }
    // Direction is down
    else if (heading > 175.0 && heading < 185.0) {
        x = (v->getX() > 48.5) ?
            (97.0 - readingsToInches(readings[0], readings[1])) 
            : (readingsToInches(readings[4], readings[5]));
        y = (v->getY() > 48.5) ?
            (97.0 - readingsToInches(readings[6], readings[7]))
            : (readingsToInches(readings[2], readings[3]));
    }
    // Direction is left
    else if (heading > 265.0 && heading < 275.0) {
        x = (v->getX() > 48.5) ?
            (97.0 - readingsToInches(readings[6], readings[7]))
            : (readingsToInches(readings[2], readings[3]));
        y = (v->getY() > 48.5) ?
            (97.0 - readingsToInches(readings[4], readings[5]))
            : (readingsToInches(readings[0], readings[1]));
    }
    // Direction is right
    else if (heading > 85.0 && heading < 95.0) {
        x = (v->getX() > 48.5) ?
            (97.0 - readingsToInches(readings[2], readings[3]))
            : (readingsToInches(readings[6], readings[7]));
        y = (v->getY() > 48.5) ?
            (97.0 - readingsToInches(readings[0], readings[1]))
            : (readingsToInches(readings[4], readings[5]));
    }
    // Otherwise, just use the vertex values
    else {
        x = v->getX();
        y = v->getY();
    }

    int tmp[] = {x, y};
    vector<double> point(tmp, tmp + sizeof(tmp) / sizeof(tmp[0]));
    return point;
}

string makeTurnCommand(vector<double> point, Vertex *end, double *heading) {
    double delta_x = end->getX() - point[0];
    double delta_y = end->getY() - point[1];
    
    // Edge case to avoid dividing by zero
    if (delta_x == 0.0) {
        (*heading) = (delta_y > 0.0) ? 0.0 : 180.0;
    }
    // Direction is right
    else if (delta_x > 0.0) {
        (*heading) = 90.0 - radiansToDegrees(atan(delta_y / delta_x));
    }
    // Otherwise, direction is left
    else {
        (*heading) = 270.0 + radiansToDegrees(atan(delta_y / delta_x));
    }

    return "2 " + to_string((*heading));
}

string makeDriveCommand(vector<double> point, Vertex *end, double heading) {
    // Determine the necesssary command:
    //  1 - Drive straight,
    //  5 - Wall-follow right, or
    //  6 - Wall-follow left
    
    int cmd;
    // Direction is up
    if ((heading > 355.0 && heading < 360.0) ||
            (heading > 0.0 && heading < 5.0))
    {
        cmd = (point[0] > 48.5) ? 5 : 6;
    }
    // Direction is down
    else if (heading > 175.0 && heading < 185.0) {
        cmd = (point[0] > 48.5) ? 6 : 5;
    }
    // Direction is left
    else if (heading > 265.0 && heading < 275.0) {
        cmd = (point[1] > 48.5) ? 5 : 6;
    }
    // Direction is right
    else if (heading > 85.0 && heading < 95.0) {
        cmd = (point[1] > 48.5) ? 6 : 5;
    }
    // Otherwise, just drive straight
    else {
        cmd = 1;
    }
    
    // Calculate the distance to drive
    double distance = sqrt(pow(end->getX() - point[0], 2)
            + pow(end->getY() - point[1], 2));
    
    return to_string(cmd) + ' ' + to_string(distance);
}

void readArduinoData(Serial8N1 *arduino, double *heading,
        vector<double> *readings)
{
    // Send command for arduino to send necessary data
    arduino->write("7 0.0");
    arduino_ready = false;
    while(arduino_ready != true);
    
    // Read received data and update IMU heading and IR readings
    (*heading) = arduino->readReal();
    for (vector<double>::iterator r = readings->begin(); r != readings->end();
            r++) (*r) = arduino->readReal();
}

// Function to handle change in level of GPIO 18
void handle_interrupt(int gpio, int level, uint32_t tick, void *flag) {
    // At interrupt rising edge, arduino is ready for a new message
    if (level == 1) {
        cout << "GPIO " << gpio
             << ": Arduino is ready to receive a command at time " << tick
             << "." << endl;
        (*(bool *)flag) = true;
    }
    // At falling edge, arduino has begun processing a command
    else {
        cout << "GPIO " << gpio
             << ": Arduino has begun processing a command at time " << tick
             << "." << endl;
    }
}
