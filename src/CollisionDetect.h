#pragma once
#include<iostream>
#include "Boid.h"
#include "ofxCv.h"
#include "Vector.h"
#include "ofxCvGrayscaleImage.h"
#include "ofxCvColorImage.h"
#include "AppConstants.h"



class CollisionDetect
{
public:

    ofxCv::ContourFinder contourFinder;
    int thresholdVal = 220;
    int minContourSize = 25;

    ofImage img  ;
    cv::Mat input;
    ofxCvGrayscaleImage cvimg;
    cv::Mat drawing=cv::Mat(IMG_HEIGHT,IMG_WIDTH,CV_8UC3,cv::Scalar(0,0,0));
    ofImage outOF;
    vector<vector<cv::Point> > contours;
    vector<cv::Rect> BBcontours;
    float simRes;
    /* public member functions*/
    void processImage();

    void boidCollision(Boid &b);
    void boidBBCollision(Boid &b);
    bool maskCollision(Boid &b,cv::Mat& mask,bool resetPos);
    void nativeContourSetup(float);
    void nativeContourFind();
    void nativeContourFind(cv::Mat&);
    void nativeDrawContours(int x,int y, int width, int height);

};


