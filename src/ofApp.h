#pragma once

#include "ofMain.h"

#undef Status
#undef STATUS

#include "ofxReprojection.h"
#include "ofxKinect.h"
#include "ofxOpenCV.h"
#include "ofxArgParser.h"
#include "opencv2/opencv.hpp"


#define WIN_WIDTH 1920*0.5
#define WIN_HEIGHT 1080*0.5

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

	ofPoint kinectMask[4];
	int maskPoints;
	cv::Mat mask;

};
