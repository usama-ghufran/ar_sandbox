#ifndef IMGPROC_H_INCLUDED
#define IMGPROC_H_INCLUDED

#include "ofxCv.h"

class ImgProc{
public:
ofxCv::ContourFinder contourFinder;
float threshold = 0.5;
ofImage img  ;
cv::Mat input;
void processImage();

};

#endif // IMGPROC_H_INCLUDED
