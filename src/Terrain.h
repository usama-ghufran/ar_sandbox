#pragma once
#include "ofxCV.h"


class Terrain {

public:
    Terrain(){};
    void setDepthImage(cv::Mat& _depthImage);

private:
    cv::Mat depthImage;
    cv::Mat grass;
    cv::Mat water;
    cv::Mat sand;
    cv::Mat rock;
    cv::Mat snow;
    cv::Mat scene;

};
