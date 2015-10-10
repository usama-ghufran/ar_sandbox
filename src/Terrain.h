#pragma once
#include "ofxCV.h"


class Terrain {

public:
    Terrain(){

        for(int i=0;i<3;i++)
            thresh[i]=0;


    };

    void loadTerrain(string filename,float mapRezImg=3);
    void setDepthImage(cv::Mat* _depthImage);
    cv::Mat getTerrain();
    void setThreshold(int threshID, int threshVal);
    void processScene();
    cv::Mat* getMasks();

private:
    cv::Mat* depthImage;
    cv::Mat grass;
    cv::Mat water;
    cv::Mat sand;
    cv::Mat rock_snow;
    cv::Mat pebble;

    cv::Mat composite;

    int thresh[3];
    cv::Mat mask[3];

};
