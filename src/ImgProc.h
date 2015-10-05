#ifndef IMGPROC_H_INCLUDED
#define IMGPROC_H_INCLUDED
#include "Boid.h"
#include "ofxCv.h"
#include"ofxCvGrayscaleImage.h"

class ImgProc{
public:

ofxCv::ContourFinder contourFinder;
float threshold = 0.5;
ofImage img  ;
cv::Mat input;
ofxCvGrayscaleImage cvimg;
/* public member functions*/
void processImage();
void boidCollision(Boid b);
};

#endif // IMGPROC_H_INCLUDED
