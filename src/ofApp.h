#pragma once

#include "ofMain.h"

#undef Status
#undef STATUS

#include "ofxReprojection.h"
#include "ofxKinect.h"
#include "ofxCV.h"
#include "ofxArgParser.h"
#include "ofxGui.h"

#include "Simulation.h"
#include "CollisionDetect.h"
#include "Terrain.h"
#include "AppConstants.h"

class ofApp : public ofBaseApp {
public:
	void setup();
	void update();
	void draw();
	void keyPressed(int key);
	void mousePressed(int x, int y, int button);
	void exit();
	void heightChanged(int& val);
    void thresh0Changed(float& val);
    void thresh1Changed(float& val);
    void thresh2Changed(float& val);
    void showContoursButtonPressed();

	ofxKinect depthcam;
	ofxReprojectionCalibration calibration;
	ofxReprojectionCalibrationData dataset;
	ofxReprojectionRenderer2D renderer;

  	vector<string> arguments;
  	bool rendererInited;
    bool fakeKinect;
    bool writeMask;
    bool guiHide;
    bool showContours;
    float mapRezSim;
    float mapRezImg;
	int maskPoints;

	Simulation sim;
	Flocking* flockDisplay;
	vector<Boid>* boids;
	CollisionDetect obj;
	Terrain scene;

	//ARGUMENTS
	int doCalib;

	cv::Mat mask;
	cv::Mat depthImage;
	cv::Mat projectImg;
    cv::Mat projectImgRGB;

    cv::Point kinectMask[4];
	cv::Point drawkinectMask[4];
    cv::Point trianglePts[3];
    ofImage projectImgOF;

	//GUI
	ofxPanel terrainControls;
	ofParameter<int> max_height;
	ofParameter<int> min_height;
	ofParameter<float> thresh0;
	ofParameter<float> thresh1;
	ofParameter<float> thresh2;
    
    ofxPanel simControls;
    ofxButton showContoursButton;
    ofParameter<float> maxSpeed;
	ofParameter<float> maxForce;
	ofParameter<float> destWeight;
	ofParameter<float> flockSeparationWeight;
	ofParameter<float> flockAlignmentWeight;
	ofParameter<float> flockCohesionWeight;
	ofParameter<float> flockSeparationRadius;
	ofParameter<float> flockAlignmentRadius;
	ofParameter<float> flockCohesionRadius;
	ofParameter<float> startRadius;
	ofParameter<float> endRadius;
    ofParameter<float> sleepTime;
    ofParameter<float> randSeed;


	//DEBUG
	cv::Mat* threshMask;
    ofImage threshMaskOF[3];

};

