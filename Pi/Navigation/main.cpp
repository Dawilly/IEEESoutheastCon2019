#include "Dijikstra.h"
#include "Graph.h"
#include "Edge.h"
#include "Vertex.h"
#include "Serial8N1.h"
#include "Camera.h"
#include <iostream>
#include <chrono>
#include <deque>
#include <string>
#include <thread>
#include <vector>
#include <math.h>
#include <pigpio.h>
#include "raspicam_cv.h"
#include "opencv2/core.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

#define PIN 18
#define PI 3.14159265358979323846
#define radiansToDegrees(theta) (theta * 180.0 / PI)

using namespace std;

vector<double> correct(vector<double> point, double heading,
        vector<double> readings, vector<double> deltas);
double wallDistance(double expected, double r1, double r2);
string makeTurnCommand(vector<double> point, Vertex *end, double *heading);
string makeDriveCommand(vector<double> point, Vertex *end, double heading);
void sendDriveCommand(raspicam::RaspiCam_Cv *camera,
       vector<bool> *debris_objects, vector<Vertex *> *corners,
       Serial8N1 *arduino, string command, vector<double> *point,
       Vertex *end, double *heading, vector<double> *readings,
       vector<double> *deltas);
void assignBaseColors(vector<Vertex *> *corners, Color color);
void readArduinoData(Serial8N1 *arduino, double *heading,
        vector<double> *readings, vector<double> *deltas);
void handle_interrupt(int gpio, int level, uint32_t tick, void *flag);

bool arduino_ready = true;

// have we recorded the initial tape color yet
bool know_home_base = false;
Color carrying_debris = Invalid;

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

    // Turn camera on and initialize vector of debris detection flags
    raspicam::RaspiCam_Cv Camera = turnCameraOn();
    vector<bool> debris_objects(8, false);
   
    // Create a graph of the playing field
    ifstream graph_file(argv[2], ifstream::in);
    vector<Vertex *> graph = makeGraph(&graph_file);
    graph_file.close();
    
    // Identify the four corners of the playing field
    vector<Vertex *> corners(4, nullptr);
    for (vector<Vertex *>::iterator c = graph.begin(); c != graph.end(); c++) {
        if ((*c)->getX() == 7.25 && (*c)->getY() == 7.25) {
            corners[0] = (*c);
        }
        else if ((*c)->getX() == 7.25 && (*c)->getY() == 89.75) {
            corners[1] = (*c);
        }
        else if ((*c)->getX() == 89.75 && (*c)->getY() == 7.25) {
            corners[2] = (*c);
        }
        else if ((*c)->getX() == 89.75 && (*c)->getY() == 89.75) {
            corners[3] = (*c);
        }
    }
    if (find(corners.begin(), corners.end(), nullptr) != corners.end()) {
        cerr << "Unable to find all four corners." << endl;
        return -1;
    }

    // Create a set of waypoints
    ifstream waypoints_file(argv[3], ifstream::in);
    vector<Vertex *> waypoints;
    while (vertexPending(&waypoints_file))
        waypoints.push_back(makeVertex(&waypoints_file, &graph));
    waypoints_file.close();

    // Initialize IMU heading, IR readings and deltas
    double heading;
    vector<double> readings(8, 0.0);
    vector<double> deltas(2, 0.0);

    // Initialize position and begin iterating through waypoints
    Vertex *position = waypoints[0];
    for (vector<Vertex *>::iterator w = waypoints.begin() + 1;
            w != waypoints.end(); w++)
    {
        // Handle irregular operation
        Vertex *target = (*w);
        if (carrying_debris != Invalid) {
            for (vector<Vertex *>::iterator c = corners.begin();
                 c != corners.end(); c++)
            {
                if ((*c)->getColor() == carrying_debris) {
                    target = (*c);
                    break;
                }
            }
        }
        cout << "Target point is (" << target->getX() << ", "
             << target->getY() << ")." << endl;

        // Use Dijikstra's algorithm to determine shortest path
        Dijikstra(graph, position);
        deque<Vertex *> path = shortestPath(position, target);
        path.pop_front(); // First vertex is position, so ignore it

        // Follow the shortest path by executing commands
        Vertex *start = position, *end = NULL;
        double tmp[] = {start->getX(), start->getY()};
        vector<double> point(tmp, tmp + sizeof(tmp) / sizeof(tmp[0]));
        for (; !path.empty(); path.pop_front()) {
            // Get the target vertex
            end = path.front();

            // Update IMU heading, IR readings, and deltas
            readArduinoData(&arduino, &heading, &readings, &deltas);

            // Correct the spatial spatial point with arduino data
            point = correct(point, heading, readings, deltas);
            cout << "X coordinate is " << point[0] << endl;
            cout << "Y coordinate is " << point[1] << endl;

            // Calculate and perform a turn command
            string command = makeTurnCommand(point, end, &heading);
            cout << "Command is " << command << "." << endl;
            arduino.write(command);
            arduino_ready = false;
            while (!arduino_ready);

            // Update IMU heading, IR readings, and deltas
            readArduinoData(&arduino, &heading, &readings, &deltas);

            // Correct the spatial point with arduino data
            point = correct(point, heading, readings, deltas);

            // Calculate and perform a drive command
            command = makeDriveCommand(point, end, heading);
            cout << "Command is " << command << "." << endl;
            sendDriveCommand(&Camera, &debris_objects, &corners, &arduino,
                    command, &point, end, &heading, &readings, &deltas); 
            
            // Update start
            start = end;
        }

        // Read arduino data to reset deltas for next path
        readArduinoData(&arduino, &heading, &readings, &deltas);

        // Update the position
        position = target;
        
        // Handle irregular operation
        if (carrying_debris != Invalid) {
            arduino.write("4 0");
	    while (!arduino_ready);
            carrying_debris = Invalid;
            w--;
        }
    }

    // Turn off camera, join GPIO thread, and finish
    turnCameraOff(Camera);
    gpioTerminate();
    return 0;
}

vector<double> correct(vector<double> point, double heading,
        vector<double> readings, vector<double> deltas)
{
    // Add the expected deltas to our previous point
    point[0] += deltas[0];
    point[1] += deltas[1];

    // Direction is up
    if ((heading > 355.0 && heading < 360.0) ||
            (heading > 0.0 && heading < 5.0))
    {
        point[0] = (point[0] > 48.5) ?
            (97.0 - wallDistance(97.0 - point[0], readings[0], readings[1]))
            : (wallDistance(point[0], readings[2], readings[3]));
        point[1] = (point[1] > 48.5) ?
            (97.0 - wallDistance(97.0 - point[1], readings[4], readings[5]))
            : (wallDistance(point[1], readings[6], readings[7]));
    }
    // Direction is down
    else if (heading > 175.0 && heading < 185.0) {
        point[0] = (point[0] > 48.5) ?
            (97.0 - wallDistance(97.0 - point[0], readings[2], readings[3]))
            : (wallDistance(point[0], readings[0], readings[1]));
        point[1] = (point[1] > 48.5) ?
            (97.0 - wallDistance(97.0 - point[1], readings[6], readings[7]))
            : (wallDistance(point[1], readings[4], readings[5]));
    }
    // Direction is left
    else if (heading > 265.0 && heading < 275.0) {
        point[0] = (point[0] > 48.5) ?
            (97.0 - wallDistance(97.0 - point[0], readings[6], readings[7]))
            : (wallDistance(97.0 - point[0], readings[4], readings[5]));
        point[1] = (point[1] > 48.5) ?
            (97.0 - wallDistance(97.0 - point[1], readings[0], readings[1]))
            : (wallDistance(point[1], readings[2], readings[3]));
    }
    // Direction is right
    else if (heading > 85.0 && heading < 95.0) {
        point[0] = (point[0] > 48.5) ?
            (97.0 - wallDistance(97.0 - point[0], readings[4], readings[5]))
            : (wallDistance(point[0], readings[6], readings[7]));
        point[1] = (point[1] > 48.5) ?
            (97.0 - wallDistance(97.0 - point[1], readings[2], readings[3]))
            : (wallDistance(point[1], readings[0], readings[1]));
    }
    // Otherwise, just return the point with the added deltas

    return point;
}

// Returns the distance to the wall. Accounts for invalid IR
//  measurements and unexpected objects seen by the IRs
double wallDistance(double expected, double r1, double r2) {
    // Determine the measured distance using valid values only
    double irDistance;
    if (r1 != -1.0 && r2 != -1.0) {
        irDistance = (r1 + r2) / 2.0;
    }
    else if (r1 == -1.0) {
        irDistance = r2;
    }
    else if (r2 == -1.0) {
        irDistance = r1;
    }
    else {
        irDistance = expected;
    }

    // Return the wall distance measured by the IRs if it is about what is
    //  expected. Otherwise, an unexpected object was seen.
    return (abs(irDistance - expected) > 12.0) ? expected : irDistance;
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

void sendDriveCommand(raspicam::RaspiCam_Cv *camera,
        vector<bool> *debris_objects, vector<Vertex *> *corners,
        Serial8N1 *arduino, string command, vector<double> *point,
        Vertex *end, double *heading, vector<double> *readings,
        vector<double> *deltas)
{
    arduino->write(command);
    arduino_ready = false;
    Color color;
    fill(debris_objects->begin(), debris_objects->end(), false);
    while (!arduino_ready) {
        color = (Color) cameraIteration((*debris_objects), (*camera));
        if (!know_home_base && color != Invalid) {
            cout << "Assigning base colors." << endl;
            assignBaseColors(corners, color);
        }
        // Search to see if debris was found
        vector<bool>::iterator search = find(debris_objects->begin(),
                debris_objects->end(), true);
        if (search != debris_objects->end()) {
            // Use index to identify debris color
            int index = distance(debris_objects->begin(), search);
            carrying_debris = (Color) (index / 2);
            cout << "index" << index << "\tcarrying_debris: " << ((int)(carrying_debris)) << endl;
            // When debris is detected, drive straight for time and pick it up
            //  with the belt
            this_thread::sleep_for(chrono::milliseconds(500));
            arduino->write("4 1");
            while (!arduino_ready);
            
            // Update IMU heading, IR readings, and deltas
            readArduinoData(arduino, heading, readings, deltas);
            
            // Correct the spatial point with arduino data
            (*point) = correct((*point), (*heading), (*readings), (*deltas));
            
            // Calculate and perform the new drive command
            command = makeDriveCommand((*point), end, (*heading));
            arduino->write(command);
            while (!arduino_ready);
            
            // Send the new drive command recursively after collecting debris
            //sendDriveCommand(camera, debris_objects, corners, arduino,
                    //command, point, end, heading, readings, deltas);
        }
    }
}

void assignBaseColors(vector<Vertex *> *corners, Color color) {
    (*corners)[0]->setColor(color);
    (*corners)[1]->setColor((Color) ((((int) color) + 1) % 4));
    (*corners)[2]->setColor((Color) ((((int) color) - 1) % 4));
    (*corners)[3]->setColor((Color) ((((int) color) + 2) % 4));
    
    cout << "Home base is color <" << (int)(*corners)[0]->getColor() << ">."
         << endl;
    cout << "Left adjacent base is color <" << (int)(*corners)[1]->getColor()
         << ">." << endl;
    cout << "Right adjacent base is color <" << (int)(*corners)[2]->getColor()
         << ">." << endl;
    cout << "Diagonal base is color <" << (int)(*corners)[3]->getColor()
         << ">." << endl;
}

void readArduinoData(Serial8N1 *arduino, double *heading,
        vector<double> *readings, vector<double> *deltas)
{
    // Send command for arduino to send necessary data
    arduino->write("7 0.0");
    arduino_ready = false;
    while (!arduino_ready);

    // Read received data and update IMU heading, IR readings, and delta values
    (*heading) = arduino->readReal();
    for (vector<double>::iterator r = readings->begin(); r != readings->end();
            r++) (*r) = arduino->readReal();
    (*deltas)[0] = arduino->readReal();
    (*deltas)[1] = arduino->readReal();
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
