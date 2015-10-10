#pragma once

#include "ofMain.h"

#undef Status
#undef STATUS

#include "ofxReprojection.h"
#include "ofxKinect.h"
#include "ofxCV.h"
#include "ofxArgParser.h"
#include "Simulation.h"
#include "CollisionDetect.h"
#include "Terrain.h"


#define WIN_WIDTH 1920*0.5
#define WIN_HEIGHT 1080*0.5

#define IMG_WIDTH 640
#define IMG_HEIGHT 480

#define KINECT_WIDTH 640
#define KINECT_HEIGHT 480


class ofApp : public ofBaseApp {
public:
	void setup();
	void update();
	void draw();
	void keyPressed(int key);
	void mousePressed(int x, int y, int button);
	void exit();
    vector<string> arguments;

	ofxKinect depthcam;
	ofxReprojectionCalibration calibration;
	ofxReprojectionCalibrationData dataset;
	ofxReprojectionRenderer2D renderer;

  	CollisionDetect obj;
	bool rendererInited;
    int doCalib;
    bool fakeKinect;
    float mapRezSim;
    float mapRezImg;
	cv::Point kinectMask[4];
	cv::Point drawkinectMask[4];
	int maskPoints;

	cv::Mat mask;
	cv::Mat depthImage;
	cv::Mat flockImg;
    cv::Mat projectImg;
    cv::Mat projectImgRGB;
    ofImage projectImgOF;
	ofImage flockingImgOF;
	ofImage depthImageOF;

    cv::Mat* threshMask;
    ofImage* threshMaskOF;

	bool writeMask;

	Simulation sim;
	Flocking* flockDisplay;
	Terrain scene;


};
