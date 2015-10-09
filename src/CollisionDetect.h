#pragma once
#include "Boid.h"
#include "ofxCv.h"
#include"ofxCvGrayscaleImage.h"

class CollisionDetect{
public:

    ofxCv::ContourFinder contourFinder;
    int thresholdVal = 150;
    int minContourSize = 25;

    ofImage img  ;
    cv::Mat input;
    ofxCvGrayscaleImage cvimg;

    vector<vector<cv::Point> > contours;
    vector<cv::Rect> BBcontours;
    cv::Mat drawing=cv::Mat(480,640,CV_8UC3,cv::Scalar(0,0,0));

    /* public member functions*/
    void processImage();

    void boidCollision(Boid &b);
    void boidBBCollision(Boid &b);

    void nativeContourSetup();
    void nativeContourFind();
    void nativeContourFind(cv::Mat);
    void nativeDrawContours();

};


