#pragma once
#include "ofxCV.h"


class Terrain {

public:
    Terrain(){};

    void loadTerrain(string filename);
    void setDepthImage(cv::Mat& _depthImage);


private:
    cv::Mat depthImage;
    cv::Mat grass;
    cv::Mat water;
    cv::Mat sand;
    cv::Mat rock_snow;
    cv::Mat pebble;

    cv::Mat scene;

};
