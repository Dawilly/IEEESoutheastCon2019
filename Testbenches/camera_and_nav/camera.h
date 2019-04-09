#include <vector>
#include "raspicam_cv.h"
#include "opencv2/core.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

#ifndef __CAMERA_INCLUDED__
#define __CAMERA_INCLUDED__

extern void turnCameraOn();
extern void turnCameraOff();
extern void findDebris();
extern void cameraIteration(std::vector<bool> &debris_objects);
extern void cameraSetup(raspicam::RaspiCam_Cv &Camera);
extern void processImage(cv::Mat &input, cv::Mat &output, int lower[3], int upper[3]);

#endif
