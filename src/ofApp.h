#pragma once

#include "ofMain.h"

#undef Status
#undef STATUS

#include "ofxReprojection.h"
#include "ofxKinect.h"
#include "ofxCV.h"
#include "ofxArgParser.h"
#include "Simulation.h"
#include "ImgProc.h"

#define WIN_WIDTH 1366//1920
#define WIN_HEIGHT 768//1080

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

    ImgProc obj;
	bool rendererInited;
    int doCalib;
    bool fakeKinect;
    float mapRez;
	cv::Point kinectMask[4];
	cv::Point drawkinectMask[4];
	int maskPoints;
	cv::Mat mask;
	cv::Mat depthImage;
	cv::Mat depthImageThresh;
	cv::Mat flockImg;

	ofImage flockingImgOF;
	ofImage depthView;
    int thresh;

	bool writeMask;

	Simulation sim;
	Flocking* flockDisplay;

};
