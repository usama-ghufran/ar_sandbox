#include "Terrain.h"


void Terrain::loadTerrain(string filename,float mapRezImg)
{
    ofBuffer terrainFile = ofBufferFromFile(ofToDataPath(filename));

    //cout<<"\nTerrain File:\n"<<terrainFile.getText()<<endl;

    grass = cv::imread(ofToDataPath(terrainFile.getNextLine()),CV_LOAD_IMAGE_COLOR);
    sand = cv::imread(ofToDataPath(terrainFile.getNextLine()),CV_LOAD_IMAGE_COLOR);
    water = cv::imread(ofToDataPath(terrainFile.getNextLine()),CV_LOAD_IMAGE_COLOR);
    pebble = cv::imread(ofToDataPath(terrainFile.getNextLine()),CV_LOAD_IMAGE_COLOR);
    rock_snow = cv::imread(ofToDataPath(terrainFile.getNextLine()),CV_LOAD_IMAGE_COLOR);

    composite=cv::Mat(480*mapRezImg,640*mapRezImg,CV_8UC3,cv::Scalar(0,0,0));

    if(!grass.data)            grass=cv::Mat(480*mapRezImg,640*mapRezImg,CV_8UC3,cv::Scalar(0,0,0));
    if(!sand.data)             sand=cv::Mat(480*mapRezImg,640*mapRezImg,CV_8UC3,cv::Scalar(0,0,0));
    if(!water.data)            water=cv::Mat(480*mapRezImg,640*mapRezImg,CV_8UC3,cv::Scalar(0,0,0));
    if(!pebble.data)           pebble=cv::Mat(480*mapRezImg,640*mapRezImg,CV_8UC3,cv::Scalar(0,0,0));
    if(!rock_snow.data)        rock_snow=cv::Mat(480*mapRezImg,640*mapRezImg,CV_8UC3,cv::Scalar(0,0,0));

    for(int i=0;i<3;i++)
        mask[i]=cv::Mat(480,640,CV_8UC3,cv::Scalar(0,0,0));
    //cv::imwrite("test.png",grass);
}

void Terrain::setDepthImage(cv::Mat* _depthImage)
{
    depthImage=_depthImage;

}

void Terrain::setThreshold(int threshID, int threshVal)
{
    thresh[threshID]=threshVal;
}

void Terrain::processScene()
{
    cv::threshold(*depthImage,mask[0],thresh[0],255,0);
    for(int i=1;i<3;i++)
    {
        cv::threshold(*depthImage,mask[i],thresh[i],255,3);
        cv::threshold(mask[i],mask[i],thresh[i-1],255,1);
    }

}

cv::Mat* Terrain::getMasks()
{
    return mask;
}

cv::Mat Terrain::getTerrain()
{
    composite=grass;

    return composite;
}
