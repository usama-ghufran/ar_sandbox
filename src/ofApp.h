#pragma once

#include "ofMain.h"

#undef Status
#undef STATUS

#include "ofxReprojection.h"
#include "ofxKinect.h"
#include "ofxCV.h"
#include "ofxArgParser.h"
#include "Simulation.h"

#define WIN_WIDTH 1920
#define WIN_HEIGHT 1080

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

	bool rendererInited;
    int doCalib;

	cv::Point kinectMask[4];
	cv::Point drawkinectMask[4];
	int maskPoints;
	cv::Mat mask;
	float mapRez;
	cv::Mat flockImg;
	ofImage flockingImgOF;

	bool writeMask;

	Simulation sim;
	Flocking* flockDisplay;

};
