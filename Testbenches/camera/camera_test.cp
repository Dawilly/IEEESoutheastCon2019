#include <iostream>
#include <ctime>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include "raspicam_cv.h"
#include "opencv2/core.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"
#include "<opencv2/imgproc.hpp"
using namespace std;

void cameraSetup(raspicam::RaspiCam_Cv &Camera);
void processImage(Mat &input, Mat &output, int lower[3], int upper[3]);

int main ( int argc,char **argv ) {
	//in order of {hue, saturation, value}
	int green_lower[3] = {40,50,50};
	int green_upper [3]= {64,255,255};

	int blue_lower[3] = {65,50,50};
	int blue_upper[3] = {130,255,255};

	int red_lower[3] = {0,50,50};
	int red_upper[3] = {10,255,255};

	int yellow_lower[3] = {10,50,50};
	int yellow_upper[3] = {40,255,255};

	int white_sensitivity = 100;
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
	cv::Mat red, blue, red, yellow, white;

	vector<vector<cv::Point>>green_contours;
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
		cv::cvtColor(blurred, hsv, CV_BGR2HSV);

		processImage(hsv, red, green_lower, green_upper);
		processImage(hsv, blue, blue_lower, blue_upper);
		processImage(hsv, red, red_lower, red_upper);
		processImage(hsv, yellow, yellow_lower, yellow_upper);
		processImage(hsv, white, white_lower, white_upper);

		cv::findContours(red, green_contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
		cv::findContours(blue, blue_contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
		cv::findContours(red, red_contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
		cv::findContours(yellow, yellow_contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
		cv::findContours(white, white_contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

	    if (green_contours.size() > 0) {
		    for (int i = 0; i < green_contours.size(); i++) {
			    epsilon = 0.01*cv::arcLength(green_contours[i], true);
			    cv::approxPolyDP(green_contours[i], approx, epsilon, true);
			    cv::Moments m = moments(thr,true);
				cv::Point p(m.m10/m.m00, m.m01/m.m00);
			    double area = cv::contourArea(approx);
			    double perimeter = cv::arcLength(approx, true);
			    if(area > 5000) {
			    	drawContours(image, green_contours, -1, cv::Scalar(0,0,0), 2);
			    	if(perimeter > 10000) {
			    		cout << "{Green Tape} " << " area: " << area << " perimeter: " << perimeter << endl;
			    		putText(image, "red tape", p, cv::FONT_HERSHEY_SIMPLEX, 1, Scalar(0,0,0), 2);
			    	}
			    	else{
			    		if(approx.size() > 10) {
			    			cout << "{Green Ball} " << " area: " << area << " perimeter: " << perimeter << endl;
			    			putText(image, "red ball", p, cv::FONT_HERSHEY_SIMPLEX, 1, Scalar(0,0,0), 2);
			    		}
			    		else {
			    			cout << "{Green Block} " << " area: " << area << " perimeter: " << perimeter << endl;
			    			putText(image, "red block", p, cv::FONT_HERSHEY_SIMPLEX, 1, Scalar(0,0,0), 2);
			    		}
			    	}
			    }
			}
	    }

	  //   if (blue_contours.size() > 0) {
		 //    for (int i = 0; i < blue_contours.size(); i++) {
			//     epsilon = 0.01*cv::arcLength(blue_contours[i], true);
			//     cv::approxPolyDP(blue_contours[i], approx, epsilon, true);
			//     cv::Moments m = moments(thr,true);
			// 	cv::Point p(m.m10/m.m00, m.m01/m.m00);
			//     double area = cv::contourArea(approx);
			//     double perimeter = cv::arcLength(approx, true);
			//     if(area > 5000) {
			//     	drawContours(image, blue_contours, -1, cv::Scalar(0,0,0), 2);
			//     	if(perimeter > 10000) {
			//     		cout << "{Blue Tape} " << " area: " << area << " perimeter: " << perimeter << endl;
			//     		putText(image, "blue tape", p, cv::FONT_HERSHEY_SIMPLEX, 1, Scalar(0,0,0), 2);
			//     	}
			//     	else{
			//     		if(approx.size() > 10) {
			//     			cout << "{Blue Ball} " << " area: " << area << " perimeter: " << perimeter << endl;
			//     			putText(image, "blue ball", p, cv::FONT_HERSHEY_SIMPLEX, 1, Scalar(0,0,0), 2);
			//     		}
			//     		else {
			//     			cout << "{Blue Block} " << " area: " << area << " perimeter: " << perimeter << endl;
			//     			putText(image, "blue block", p, cv::FONT_HERSHEY_SIMPLEX, 1, Scalar(0,0,0), 2);
			//     		}
			//     	}
			//     }
			// }
	  //   }

	  //   if (red_contours.size() > 0) {
		 //    for (int i = 0; i < red_contours.size(); i++) {
			//     epsilon = 0.01*cv::arcLength(red_contours[i], true);
			//     cv::approxPolyDP(red_contours[i], approx, epsilon, true);
			//     cv::Moments m = moments(thr,true);
			// 	cv::Point p(m.m10/m.m00, m.m01/m.m00);
			//     double area = cv::contourArea(approx);
			//     double perimeter = cv::arcLength(approx, true);
			//     if(area > 5000) {
			//     	drawContours(image, red_contours, -1, cv::Scalar(0,0,0), 2);
			//     	if(perimeter > 10000) {
			//     		cout << "{Red Tape} " << " area: " << area << " perimeter: " << perimeter << endl;
			//     		putText(image, "red tape", p, cv::FONT_HERSHEY_SIMPLEX, 1, Scalar(0,0,0), 2);
			//     	}
			//     	else{
			//     		if(approx.size() > 10) {
			//     			cout << "{Red Ball} " << " area: " << area << " perimeter: " << perimeter << endl;
			//     			putText(image, "red ball", p, cv::FONT_HERSHEY_SIMPLEX, 1, Scalar(0,0,0), 2);
			//     		}
			//     		else {
			//     			cout << "{Green Block} " << " area: " << area << " perimeter: " << perimeter << endl;
			//     			putText(image, "red block", p, cv::FONT_HERSHEY_SIMPLEX, 1, Scalar(0,0,0), 2);
			//     		}
			//     	}
			//     }
			// }
	  //   }

	  //   if (red_contours.size() > 0) {
		 //    for (int i = 0; i < red_contours.size(); i++) {
			//     epsilon = 0.01*arcLength(red_contours[i], True);
			//     approx = approxPolyDP(red_contours[i], approx, epsilon, True);
			//     if contourArea(approx) > 19000 {
			//         drawContours(image, [approx], -1, (0,0, 255), 2);
			//     }
			// 	if(contourArea(approx) > 50000) {
			// 		print("Red Tape");
			// 	}
			// 	else {
			// 		print("Red Debris");
			// 	}
			// }
	  //   }

	  //   if (yellow_contours.size() > 0) {
		 //    for (int i = 0; i < yellow_contours.size(); i++) {
			//     epsilon = 0.01*arcLength(yellow_contours[i], True);
			//     approx = approxPolyDP(yellow_contours[i], approx, epsilon, True);
			//     if contourArea(approx) > 19000 {
			//         drawContours(image, [approx], -1, (0,0, 255), 2);
			//     }
			// 	if(contourArea(approx) > 50000) {
			// 		print("Yellow Tape");
			// 	}
			// 	else {
			// 		print("Yellow Debris");
			// 	}
			// }
	  //   }

	  //   if (white_contours.size() > 0) {
		 //    for (int i = 0; i < white_contours.size(); i++) {
			//     epsilon = 0.01*arcLength(white_contours[i], True);
			//     approx = approxPolyDP(white_contours[i], approx, epsilon, True);
			//     if contourArea(approx) > 19000 {
			//         drawContours(image, [approx], -1, (0,0, 255), 2);
			//     }
			// 	if(contourArea(approx) > 50000) {
			// 		print("White Tape");
			// 	}
			// 	else {
			// 		print("White Debris");
			// 	}
			// }
	  //   }


		imshow("Camera Stream", image);
		if(waitKey(5) >= 0) {
			break;
		}
	}


	Camera.release();
	return 0;
}

//TO DO: adjust these values for best possible image
void cameraSetup(raspicam::RaspiCam_Cv &Camera) {
	Camera.set ( cv::CAP_PROP_FRAME_WIDTH,  640.0 );
    Camera.set ( cv::CAP_PROP_FRAME_HEIGHT, 480.0 );
    Camera.set ( cv::CAP_PROP_BRIGHTNESS, 50.0 );
    Camera.set ( cv::CAP_PROP_CONTRAST , 50.0 );
    Camera.set ( cv::CAP_PROP_SATURATION, 50.0 );
    Camera.set ( cv::CAP_PROP_GAIN, 50.0 );
    Camera.set ( cv::CAP_PROP_FPS, 32.0 );
}

void processImage(Mat &input, Mat &output, int lower[3], int upper[3]) {
	inRange(input, Scalar(lower[0], lower[1], lower[2]), Scalar(upper[0], upper[1], upper[2]), output);
	erode(output, output, iterations = 2);
	dilate(output, output, iterations = 2);	
}