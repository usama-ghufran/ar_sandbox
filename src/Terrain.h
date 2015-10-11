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
    void setThreshold(int threshID, int threshVal);
    void processScene();
    void applyThreshold();
    void multMapWithMask(cv::Mat img,cv::Mat mask,cv::Mat& dest);
    cv::Mat* getMasks();
    cv::Mat getTerrain();
    void combineMapsWithMasks();
    void combineMapsWithMasks2();

private:
    cv::Mat* depthImage;
    cv::Mat grass;
    cv::Mat water;
    cv::Mat sand;
    cv::Mat rock_snow;
    cv::Mat pebble;
    cv::Mat composite;
    cv::Mat mask[4];
    cv::Mat* textures[4];
    int thresh[3];
    float mapRez;


};
