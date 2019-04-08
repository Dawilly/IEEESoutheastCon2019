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
string makeDriveCommand(vector<double> point, Vertex *end, double heading);
void readArduinoData(Serial8N1 *arduino, double *heading,
        vector<double> *readings);
void handle_interrupt(int gpio, int level, uint32_t tick, void *flag);
void cameraSetup(raspicam::RaspiCam_Cv &Camera);
void processImage(cv::Mat &input, cv::Mat &output, int lower[3], int upper[3]);
void runCamera();

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
        waypoints.push_back(makeVertex(&waypoints_file, &graph));
    waypoints_file.close();

    // Initialize IMU heading and IR readings
    double heading;
    vector<double> readings(8, 0.0);
    
    //begin camera stream
    thread first(runCamera);

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
            while(arduino_ready != true);

            // Update IMU heading and IR readings
            readArduinoData(&arduino, &heading, &readings);

            // Update spatial point with arduino data
            point = correct(point, heading, readings);

            // Calculate and perform a drive command
            command = makeDriveCommand(point, end, heading);
            cout << "Command is " << command << "." << endl;
            arduino.write(command);
            arduino_ready = false;
            while(arduino_ready != true);

            // Update start
            start = end;
        }

        // Update the position
        position = (*w);
    }

    // Join Camera thread
    first.join();
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

void runCamera() {
	//in order of {hue, saturation, value}
	int green_lower[3] = {45,100,80};
	int green_upper [3]= {84,255,255};

	int blue_ball_lower[3] = {100,150,10};
	int blue_ball_upper[3] = {140,255,255};

	int blue_lower[3] ={85,10,100};
	int blue_upper[3] ={103,255,255};	

	int red_lower[3] = {155,80,100};
	int red_upper[3] = {180,255,255};

	int yellow_lower[3] = {25,75,125};
	int yellow_upper[3] = {35,255,255};

	int white_sensitivity = 125;
	int white_lower[3] = {0,0,255-white_sensitivity};
	int white_upper[3] = {255,white_sensitivity,255};

	//doing it using raspbery pi camera API
	raspicam::RaspiCam_Cv Camera;
	cameraSetup(Camera);

	cout <<"Connecting to camera" << endl;
	if(!Camera.open()) {
		cout << "Oops! Error opening camera." << endl;
		return 0;
	}
	cout << "Connected to camera = " << Camera.getId() << endl;

	cv::Mat image, hsv, blurred;
	cv::Mat green, blue, blue_ball, red, yellow, white;

	vector<vector<cv::Point>>green_contours;
	vector<vector<cv::Point>>blue_ball_contours;
	vector<vector<cv::Point>>blue_contours;
	vector<vector<cv::Point>>red_contours;
	vector<vector<cv::Point>>yellow_contours;
        vector<vector<cv::Point>>white_contours;
        vector<cv::Vec4i> hierarchy;

        vector<cv::Point> approx;

        double epsilon;

	cout << "Capturing" << endl;

	//maybe change to while(!3MinutesPassed) ?
	while(1) {
		Camera.grab();
		Camera.retrieve(image);

		cv::GaussianBlur(image, blurred, cv::Size(11,11), 0);
		cv::cvtColor(blurred, hsv, cv::COLOR_BGR2HSV);

		processImage(hsv, green, green_lower, green_upper);
		processImage(hsv, blue, blue_lower, blue_upper);
		processImage(hsv, blue_ball, blue_ball_lower, blue_ball_upper);
		processImage(hsv, red, red_lower, red_upper);
		processImage(hsv, yellow, yellow_lower, yellow_upper);
		processImage(hsv, white, white_lower, white_upper);

		cv::findContours(green, green_contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
		cv::findContours(blue, blue_contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
		cv::findContours(blue_ball, blue_ball_contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
		cv::findContours(red, red_contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
		cv::findContours(yellow, yellow_contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
		cv::findContours(white, white_contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

		if (green_contours.size() > 0) {
			for (int i = 0; i < green_contours.size(); i++) {
				epsilon = 0.01*cv::arcLength(green_contours[i], true);
				cv::approxPolyDP(green_contours[i], approx, epsilon, true);
				cv::Moments m = cv::moments(green_contours[i],true);
				cv::Point p(m.m10/m.m00, m.m01/m.m00);
				double area = cv::contourArea(approx);
				double perimeter = cv::arcLength(approx, true);
				if(area > 5000) {
					drawContours(image, green_contours, -1, cv::Scalar(0,0,0), 2);
					if(perimeter > 1000) {
						cout << "{Green Tape} " << " area: " << area << " perimeter: " << perimeter << endl;
						putText(image, "green tape", p, cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0,0,0), 2);
					}
					else{
						if(approx.size() > 10) {
							cout << "{Green Ball} " << " area: " << area << " perimeter: " << perimeter << endl;
							putText(image, "green ball", p, cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0,0,0), 2);
						}
						else {
							cout << "{Green Block} " << " area: " << area << " perimeter: " << perimeter << endl;
							putText(image, "green block", p, cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0,0,0), 2);
						}
					}
				}
			}
		}

		 if (blue_contours.size() > 0) {
			 for (int i = 0; i < blue_contours.size(); i++) {
				 epsilon = 0.01*cv::arcLength(blue_contours[i], true);
				 cv::approxPolyDP(blue_contours[i], approx, epsilon, true);
				 cv::Moments m = moments(blue_contours[i],true);
				 cv::Point p(m.m10/m.m00, m.m01/m.m00);
				 double area = cv::contourArea(approx);
				 double perimeter = cv::arcLength(approx, true);
				 if(area > 5000) {
					drawContours(image, blue_contours, -1, cv::Scalar(0,0,0), 2);
					if(perimeter > 1000) {
						cout << "{Blue Tape} " << " area: " << area << " perimeter: " << perimeter << endl;
						putText(image, "blue tape", p, cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0,0,0), 2);
					}
					else{
						cout << "{Blue Block} " << " area: " << area << " perimeter: " << perimeter << endl;
						putText(image, "blue block", p, cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0,0,0), 2);
					}
				 }
			 }
		 }

		if (blue_ball_contours.size() > 0) {
			 for (int i = 0; i < blue_ball_contours.size(); i++) {
				 epsilon = 0.01*cv::arcLength(blue_ball_contours[i], true);
				 cv::approxPolyDP(blue_ball_contours[i], approx, epsilon, true);
				 cv::Moments m = moments(blue_ball_contours[i],true);
				 cv::Point p(m.m10/m.m00, m.m01/m.m00);
				 double area = cv::contourArea(approx);
				 double perimeter = cv::arcLength(approx, true);
				 if(area > 5000) {
					drawContours(image, blue_ball_contours, -1, cv::Scalar(0,0,0), 2);
					putText(image, "blue ball", p, cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255,255,255), 2);

				}
			 }
		}

		if (red_contours.size() > 0) {
			for (int i = 0; i < red_contours.size(); i++) {
				epsilon = 0.01*cv::arcLength(red_contours[i], true);
				cv::approxPolyDP(red_contours[i], approx, epsilon, true);
				cv::Moments m = cv::moments(red_contours[i],true);
				cv::Point p(m.m10/m.m00, m.m01/m.m00);
				double area = cv::contourArea(approx);
				double perimeter = cv::arcLength(approx, true);
				if(area > 5000) {
					drawContours(image, red_contours, -1, cv::Scalar(0,0,0), 2);
					if(perimeter > 1000) {
						cout << "{Red Tape} " << " area: " << area << " perimeter: " << perimeter << endl;
						putText(image, "red tape", p, cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0,0,0), 2);
					}
					else{
						if(approx.size() > 10) {
							cout << "{Red Ball} " << " area: " << area << " perimeter: " << perimeter << endl;
							putText(image, "red ball", p, cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0,0,0), 2);
						}
						else {
							cout << "{Red Block} " << " area: " << area << " perimeter: " << perimeter << endl;
							putText(image, "red block", p, cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0,0,0), 2);
						}
					}
				}
			}
		}


		if (yellow_contours.size() > 0) {
			for (int i = 0; i < yellow_contours.size(); i++) {
				epsilon = 0.01*cv::arcLength(yellow_contours[i], true);
				cv::approxPolyDP(yellow_contours[i], approx, epsilon, true);
				cv::Moments m = cv::moments(yellow_contours[i],true);
				cv::Point p(m.m10/m.m00, m.m01/m.m00);
				double area = cv::contourArea(approx);
				double perimeter = cv::arcLength(approx, true);
				if(area > 5000) {
					drawContours(image, yellow_contours, -1, cv::Scalar(0,0,0), 2);
					if(perimeter > 1000) {
						cout << "{Yellow Tape} " << " area: " << area << " perimeter: " << perimeter << endl;
						putText(image, "yellow tape", p, cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0,0,0), 2);
					}
					else{
						if(approx.size() > 10) {
							cout << "{Yellow Ball} " << " area: " << area << " perimeter: " << perimeter << endl;
							putText(image, "yellow ball", p, cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0,0,0), 2);
						}
						else {
							cout << "{Yellow Block} " << " area: " << area << " perimeter: " << perimeter << endl;
							putText(image, "yellow block", p, cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0,0,0), 2);
						}
					}
				}
			}
		}

		if (white_contours.size() > 0) {
			for (int i = 0; i < white_contours.size(); i++) {
				epsilon = 0.01*cv::arcLength(white_contours[i], true);
				cv::approxPolyDP(white_contours[i], approx, epsilon, true);
				cv::Moments m = cv::moments(white_contours[i],true);
				cv::Point p(m.m10/m.m00, m.m01/m.m00);
				double area = cv::contourArea(approx);
				double perimeter = cv::arcLength(approx, true);
				if(area > 5000) {
					drawContours(image, white_contours, -1, cv::Scalar(0,0,0), 2);
					if(perimeter > 1000) {
						cout << "{White Tape} " << " area: " << area << " perimeter: " << perimeter << endl;
						putText(image, "white tape", p, cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0,0,0), 2);
					}
				}
			}
		}	    

		cv::imshow("Camera Stream", image);
		if(cv::waitKey(5) >= 0) {
			break;
		}
	}


	Camera.release();
	return 0;
}

void cameraSetup(raspicam::RaspiCam_Cv &Camera) {
	Camera.set ( cv::CAP_PROP_FRAME_WIDTH,  640.0 );
        Camera.set ( cv::CAP_PROP_FRAME_HEIGHT, 480.0 );
        Camera.set ( cv::CAP_PROP_BRIGHTNESS, 50.0 );
        Camera.set ( cv::CAP_PROP_CONTRAST , 50.0 );
        Camera.set ( cv::CAP_PROP_SATURATION, 50.0 );
        Camera.set ( cv::CAP_PROP_GAIN, 50.0 );
        Camera.set ( cv::CAP_PROP_FPS, 32.0 );
}

void processImage(cv::Mat &input, cv::Mat &output, int lower[3], int upper[3]) {
	inRange(input, cv::Scalar(lower[0], lower[1], lower[2]), cv::Scalar(upper[0], upper[1], upper[2]), output);
	erode(output, output, cv::Mat(), cv::Point(-1,-1), 2);
	dilate(output, output, cv::Mat(),cv::Point(-1,-1), 2);
}
