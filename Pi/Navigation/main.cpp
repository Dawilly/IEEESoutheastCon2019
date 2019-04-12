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
#include <time.h>
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
        Serial8N1 *arduino, string command);
void assignBaseColors(vector<Vertex *> *corners, Color color);
void readArduinoData(Serial8N1 *arduino, double *heading,
        vector<double> *readings, vector<double> *deltas);
void handle_interrupt(int gpio, int level, uint32_t tick, void *flag);

bool arduino_ready = true;

// have we recorded the initial tape color yet
bool know_home_base = false;
Color carrying_debris = Invalid;

int main(int argc, char **argv) {
    // Initialize the start time and timeout flag
    time_t start_time = time(0);
    bool timeout = false;

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

    // Initialize position and spatial point, begin iterating through waypoints
    Vertex *position = waypoints[0];
    double tmp[] = {position->getX(), position->getY()};
    vector<double> point(tmp, tmp + sizeof(tmp) / sizeof(tmp[0]));
    for (vector<Vertex *>::iterator w = waypoints.begin() + 1;
            w != waypoints.end(); w++)
    {
    string answer = (know_home_base) ? "Yes!" : "No.";
	cout << "Homebase identified? " << answer << endl;
        // Handle irregular operation
        Vertex *target = (*w);
        if (!timeout && carrying_debris != Invalid) {
            for (vector<Vertex *>::iterator c = corners.begin();
                 c != corners.end(); c++)
            {
                if ((*c)->getColor() == carrying_debris) {
                    target = (*c);
                    break;
                }
            }
            // We couldn't find the corresponding color, so go home
            if (target == (*w)) {
                target = corners[0];
            }
        }
        cout << "Target point is (" << target->getX() << ", "
             << target->getY() << ")." << endl;

        // Use Dijikstra's algorithm to determine shortest path
        Dijikstra(graph, position);
        deque<Vertex *> path = shortestPath(position, target);
        path.pop_front(); // First vertex is position, so ignore it

        // Follow the shortest path by executing commands
        Vertex *end = nullptr;
        for (; !path.empty(); path.pop_front()) {
            // Get the target vertex
            end = path.front();
            cout << "Aiming for vertex (" << end->getX() << ", " << end->getY()
                 << ")." << endl;

            // Update IMU heading, IR readings, and deltas
            readArduinoData(&arduino, &heading, &readings, &deltas);

            // Correct the spatial spatial point with arduino data
            point = correct(point, heading, readings, deltas);

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
            if (!know_home_base || carrying_debris == Invalid) {
                sendDriveCommand(&Camera, &debris_objects, &corners, &arduino,
                        command);
            }
            else {
                arduino.write(command);
                arduino_ready = false;
                while (!arduino_ready);
            }
            
            // Update position
            position = end;
        }

        // Handle irregular operation
        if (target != (*w)) {
            // Drive into the corner
            arduino.write("1 4.0");
	        arduino_ready = false;
            while (!arduino_ready);
            
            // Back out of the corner to deposit block
            arduino.write("1 -4.0");
	        arduino_ready = false;
            while (!arduino_ready);

            // Reset debris flag and decrement waypoint iterator
            carrying_debris = Invalid;
            w--;
        }

        // Determine if we are running out of time and need to go home
        time_t current_time = time(0);
        if (!timeout && current_time - start_time >= 160) {
            // Set the waypoint itertor to our last waypoint for the next
            //  iteration
            w = waypoints.end() - 2;
            
            // Update the start time to see how long it takes us to get home
            start_time = current_time;
            
            // Timeout occured
            timeout = true;
        }
    }
    
    // Inidicate the time of run in seconds
    if (timeout) {
        cout << "After timeout, we got home in " << time(0) - start_time
             << " seconds." << endl;
    }
    else {
        cout << "Total run was " << time(0) - start_time << " seconds."
             << endl;
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
    cout << "Correcting spatial point." << endl;
    point[0] += deltas[0];
    point[1] += deltas[1];
    cout << "Delta X: " << deltas[0] << endl;
    cout << "Delta Y: " << deltas[1] << endl;

    // Direction is up
    if ((heading >= 355.0 && heading <= 360.0) ||
            (heading >= 0.0 && heading <= 5.0))
    {
        cout << "Using IR sensors . . . " << endl;
        point[0] = (point[0] > 48.5) ?
            (97.0 - wallDistance(97.0 - point[0], readings[0], readings[1]))
            : (wallDistance(point[0], readings[2], readings[3]));
        point[1] = (point[1] > 48.5) ?
            (97.0 - wallDistance(97.0 - point[1], readings[4], readings[5]))
            : (wallDistance(point[1], readings[6], readings[7]));
    }
    // Direction is down
    else if (heading >= 175.0 && heading <= 185.0) {
        cout << "Using IR sensors . . . " << endl;
        point[0] = (point[0] > 48.5) ?
            (97.0 - wallDistance(97.0 - point[0], readings[2], readings[3]))
            : (wallDistance(point[0], readings[0], readings[1]));
        point[1] = (point[1] > 48.5) ?
            (97.0 - wallDistance(97.0 - point[1], readings[6], readings[7]))
            : (wallDistance(point[1], readings[4], readings[5]));
    }
    // Direction is left
    else if (heading >= 265.0 && heading <= 275.0) {
        cout << "Using IR sensors . . . " << endl;
        point[0] = (point[0] > 48.5) ?
            (97.0 - wallDistance(97.0 - point[0], readings[6], readings[7]))
            : (wallDistance(point[0], readings[4], readings[5]));
        point[1] = (point[1] > 48.5) ?
            (97.0 - wallDistance(97.0 - point[1], readings[0], readings[1]))
            : (wallDistance(point[1], readings[2], readings[3]));
    }
    // Direction is right
    else if (heading >= 85.0 && heading <= 95.0) {
        cout << "Using IR sensors . . . " << endl;
        point[0] = (point[0] > 48.5) ?
            (97.0 - wallDistance(97.0 - point[0], readings[4], readings[5]))
            : (wallDistance(point[0], readings[6], readings[7]));
        point[1] = (point[1] > 48.5) ?
            (97.0 - wallDistance(97.0 - point[1], readings[2], readings[3]))
            : (wallDistance(point[1], readings[0], readings[1]));
    }
    // Otherwise, just return the point with the added deltas
    cout << "X coordinate is " << point[0] << endl;
    cout << "Y coordinate is " << point[1] << endl;
    cout << "Heading is " << heading << endl;

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
        cout << "IRs are out of range, returning expected position."
             << endl;
        irDistance = expected;
    }

    // Return the wall distance measured by the IRs if it is about what is
    //  expected. Otherwise, an unexpected object was seen.
    if (abs(irDistance - expected) > 12.0) {
        cout << "IR readings are inaccurate, returning expected position."
             << endl;
    }
    return (abs(irDistance - expected) > 12.0) ? expected : irDistance;
}

string makeTurnCommand(vector<double> point, Vertex *end, double *heading) {
    double delta_x = end->getX() - point[0];
    double delta_y = end->getY() - point[1];

    (*heading) = (-atan2(delta_y, delta_x) + PI / 2.0) * 180.0 / PI;
    if ((*heading) < 0.0) (*heading) += 360.0;

    return "2 " + to_string((*heading));
}

string makeDriveCommand(vector<double> point, Vertex *end, double heading) {
    // Determine the necesssary command:
    //  1 - Drive straight,
    //  5 - Wall-follow right, or
    //  6 - Wall-follow left

    int cmd;
    // Direction is up
    if ((heading >= 355.0 && heading <= 360.0) ||
            (heading >= 0.0 && heading <= 5.0))
    {
        cmd = (point[0] > 48.5) ? 5 : 6;
    }
    // Direction is down
    else if (heading >= 175.0 && heading <= 185.0) {
        cmd = (point[0] > 48.5) ? 6 : 5;
    }
    // Direction is left
    else if (heading >= 265.0 && heading <= 275.0) {
        cmd = (point[1] > 48.5) ? 5 : 6;
    }
    // Direction is right
    else if (heading >= 85.0 && heading <= 95.0) {
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
        Serial8N1 *arduino, string command)
{
    arduino->write(command);
    arduino_ready = false;
    Color color;
    fill(debris_objects->begin(), debris_objects->end(), false);
    while (!arduino_ready) {
        color = (Color) cameraIteration((*debris_objects), (*camera));
        
        // If we now know our home base, then assign base colors
        if (!know_home_base && color != Invalid) {
            cout << "Assigning base colors." << endl;
            assignBaseColors(corners, color);
            know_home_base = true;
        }

        // Search to see if debris was found in front of us
        vector<bool>::iterator search = find(debris_objects->begin(),
                debris_objects->end(), true);
        
        // If there is debris in front of us, and we aren't holding some, pick
        //  it up
        if (search != debris_objects->end() && carrying_debris == Invalid) {
            // Use index to identify debris color
            int index = distance(debris_objects->begin(), search);
            carrying_debris = (Color) (index / 2);
            cout << "Moving debris of color <" << carrying_debris << ">."
                 << endl;

            /*
            // When debris is detected, drive straight for time and pick it up
            //  with the belt
            this_thread::sleep_for(chrono::seconds(2));
            arduino->write("4 1");
            arduino_ready = false;
            while (!arduino_ready);
            
            // Update IMU heading, IR readings, and deltas
            readArduinoData(arduino, heading, readings, deltas);
            
            // Correct the spatial point with arduino data
            (*point) = correct((*point), (*heading), (*readings), (*deltas));
            
            // Calculate and perform the new drive command
            command = makeDriveCommand((*point), end, (*heading));
            arduino->write(command);
            arduino_ready = false;
            while (!arduino_ready);
            */
            
            // Send the new drive command recursively after collecting debris
            //sendDriveCommand(camera, debris_objects, corners, arduino,
                    //command, point, end, heading, readings, deltas);
        }
    }
}

//corners goes: home, left, right, diagonal
void assignBaseColors(vector<Vertex *> *corners, Color color) {
    (*corners)[0]->setColor((Color) ((((int) color) + 1) % 4));
    (*corners)[1]->setColor((Color) ((((int) color) + 2) % 4));
    (*corners)[2]->setColor(color);
    (*corners)[3]->setColor((Color) ((((int) color) - 1) % 4));
    
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
    cout << "Waiting on data . . ." << endl;
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
