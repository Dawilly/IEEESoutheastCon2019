#include "Dijikstra.h"
#include "Graph.h"
#include "Edge.h"
#include "Vertex.h"
#include "Serial8N1.h"
#include "camera.h"
#include <iostream>
#include <deque>
#include <string>
#include <thread>
#include <vector>
#include <math.h>
#include <pigpio.h>
#include <iostream>
#include <ctime>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include "raspicam_cv.h"
#include "opencv2/core.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

#define PIN 18
inline double readingsToInches(double r1, double r2) {
    return ((r1 + r2) / 25.4 / 2.0) + 4.5;
}
#define PI 3.14159265358979323846
#define radiansToDegrees(theta) (theta * 180.0 / PI)

using namespace std;

vector<double> correct(vector<double> point, double heading, vector<double> readings);
string makeTurnCommand(vector<double> point, Vertex *end, double *heading);
void sendDriveCommand(string command, vector<double> point, Vertex *end, 
		      double heading, vector<bool> debris_objects, 
		      Serial8N1 &arduino, vector<double> readings, raspicam::RaspiCam_Cv &Camera,
              vector<Vertex *> &vertices);
string makeDriveCommand(vector<double> point, Vertex *end, double heading);
void readArduinoData(Serial8N1 *arduino, double *heading,
        vector<double> *readings);
void handle_interrupt(int gpio, int level, uint32_t tick, void *flag);
void cameraSetup(raspicam::RaspiCam_Cv &Camera);
void processImage(cv::Mat &input, cv::Mat &output, int lower[3], int upper[3]);

bool arduino_ready = true;

// the number of debris objects collected 
int debris_collected = 0;
// have we recorded the initial tape color yet
bool know_home_base = false;

int main(int argc, char **argv) {
    // Check number of arguments
    if (argc < 4) {
        cerr << "Too few arguments!" << endl;
        return -1;
    }
    
    vector<bool> debris_objects(8, false);

    // Set up GPIO 18 as "interrupt" to indicate arduino is ready for a message
    gpioInitialise();
    gpioSetMode(PIN, PI_INPUT);
    gpioSetPullUpDown(PIN, PI_PUD_UP);

    // Pass function to handle interrupt at rising and falling edges
    gpioSetISRFuncEx(PIN, EITHER_EDGE, 0, handle_interrupt, &arduino_ready);

    // Open serial port and use IRs to be parallel with left wall
    Serial8N1 arduino(argv[1], 9600);

    //turn camera on
    raspicam::RaspiCam_Cv Camera = turnCameraOn();
    
    // Create a graph of the playing field
    ifstream graph_file(argv[2], ifstream::in);
    vector<Vertex *> graph = makeGraph(&graph_file);
    graph_file.close();

    // Create a set of waypoints
    ifstream waypoints_file(argv[3], ifstream::in);
    vector<Vertex *> waypoints;
    while (vertexPending(&waypoints_file)) {
        waypoints.push_back(makeVertex(&waypoints_file, &graph));
    }
    waypoints_file.close();

    // Initialize IMU heading and IR readings
    double heading;
    vector<double> readings(8, 0.0);

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

            // Update IMU heading and IR readings
            readArduinoData(&arduino, &heading, &readings);

            // Initialize spatial point and correct with arduino data
            double tmp[] = {start->getX(), start->getY()};
            vector<double> point(tmp, tmp + sizeof(tmp) / sizeof(tmp[0]));
            point = correct(point, heading, readings);

            // Calculate and perform a turn command
            string command = makeTurnCommand(point, end, &heading);
            cout << "Command is " << command << "." << endl;
            arduino.write(command);
            arduino_ready = false;
            while(arduino_ready != true) {
                //turn
            }
	    
    	    // Update IMU heading and IR readings
    	    readArduinoData(&arduino, &heading, &readings);

    	    // Update spatial point with arduino data
    	    point = correct(point, heading, readings);
    	    
    	    // Calculate and perform a drive command
    	    command = makeDriveCommand(point, end, heading);
    	    cout << "Command is " << command << "." << endl;
    	    sendDriveCommand(command, point, end, heading, debris_objects, arduino, readings, Camera, graph);

            // Update start
            start = end;
        }

        // Update the position
        position = (*w);
    }

    //turn off camera
    turnCameraOff(Camera);
    // Join GPIO thread and finish
    gpioTerminate();
    return 0;
}

vector<double> correct(vector<double> point, double heading, vector<double> readings) {
    // Direction is up
    if ((heading > 355.0 && heading < 360.0) ||
            (heading > 0.0 && heading < 5.0))
    {
        point[0] = (point[0] > 48.5) ?
            (97.0 - readingsToInches(readings[0], readings[1]))
            : (readingsToInches(readings[2], readings[3]));
        //point[1] = (point[1] > 48.5) ?
            //(97.0 - readingsToInches(readings[2], readings[3]))
            //: (readingsToInches(readings[6], readings[7]));
    }
    // Direction is down
    else if (heading > 175.0 && heading < 185.0) {
        point[0] = (point[0] > 48.5) ?
            (97.0 - readingsToInches(readings[2], readings[3]))
            : (readingsToInches(readings[0], readings[1]));
        //point[1] = (point[1] > 48.5) ?
            //(97.0 - readingsToInches(readings[6], readings[7]))
            //: (readingsToInches(readings[2], readings[3]));
    }
    // Direction is left
    else if (heading > 265.0 && heading < 275.0) {
        //point[0] = (point[0] > 48.5) ?
            //(97.0 - readingsToInches(readings[6], readings[7]))
            //: (readingsToInches(readings[2], readings[3]));
        point[1] = (point[1] > 48.5) ?
            (97.0 - readingsToInches(readings[0], readings[1]))
            : (readingsToInches(readings[2], readings[3]));
    }
    // Direction is right
    else if (heading > 85.0 && heading < 95.0) {
        //x = (point[0] > 48.5) ?
            //(97.0 - readingsToInches(readings[2], readings[3]))
            //: (readingsToInches(readings[6], readings[7]));
        point[1] = (point[1] > 48.5) ?
            (97.0 - readingsToInches(readings[2], readings[3]))
            : (readingsToInches(readings[0], readings[1]));
    }
    // Otherwise, just use original point values (do nothing)

    return point;
}

void sendDriveCommand(string command, vector<double> point, Vertex *end, double heading, 
		      vector<bool> debris_objects, Serial8N1 &arduino, vector<double> readings, raspicam::RaspiCam_Cv &Camera, vector<Vertex *> &vertices) {
    arduino.write(command);
    arduino_ready = false;
    Color color = Invalid;
    fill(debris_objects.begin(), debris_objects.end(), false);
    while(arduino_ready != true){
    	color = (Color)(cameraIteration(debris_objects, Camera));
        if(!know_home_base) {
            assignBaseColors(vertices, (Color)(color));
        }
    	if(find(debris_objects.begin(), debris_objects.end(), true) != debris_objects.end()) {
    	    //debris_objects contains true => there is debris in front of us
    	    string new_command = "4 1";
    	    arduino.write(new_command);
    	    while(arduino_ready != true) {
    		// pick up block
    	    }
    	    
    	    // Update IMU heading and IR readings
    	    readArduinoData(&arduino, &heading, &readings);

    	    // Update spatial point with arduino data
    	    point = correct(point, heading, readings);
    	    
    	    // Calculate and perform a drive command
    	    command = makeDriveCommand(point, end, heading);
    	    
    	    sendDriveCommand(command, point, end, heading, debris_objects, arduino, readings, Camera);
    	}
    }
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

void readArduinoData(Serial8N1 *arduino, double *heading, vector<double> *readings) {
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

void assignBaseColors(vector<Vertex *> &vertices, Color color) {
    Vertex *home_base = NULL;
    Vertex *left_adj = NULL;
    Vertex *right_adj = NULL;
    Vertex *diagonal = NULL;
    double hb_x, hb_y, la_x, la_y, ra_x, ra_y, d_x, d_y;
    hb_x = 7.25;
    hb_y = 7.25;
    la_x = 7.25;
    la_y = 89.75;
    ra_x = 89.75;
    ra_y = 7.25;
    d_x = 89.75;
    d_y = 89.75;
    int base_count = 0;
    for (vector<Vertex *>::iterator i = vertices.begin(); i != vertices.end(); i++) {
        if ((*i)->getX() == hb_x && (*i)->getY() == hb_y) {
            home_base = (*i);
            base_count++;
        }
        else if(((*i)->getX() == la_x && (*i)->getY() == la_y)) {
            left_adj = (*i);
            base_count++;
        }
        else if(((*i)->getX() == ra_x && (*i)->getY() == ra_y)) {
            right_adj = (*i);
            base_count++;
        }
        else if(((*i)->getX() == d_x && (*i)->getY() == d_y)) {
            diagonal = (*i);
            base_count++;
        }

        if(base_count == 4) {
            break;
        }
    }

    home_base->setColor(color);
    left_adj->setColor((Color)((((int)(color)) + 1) % 4));
    right_adj->setColor((Color)((((int)(color)) - 1) % 4));
    diagonal->setColor((Color)((((int)(color)) + 2) % 4));
}

