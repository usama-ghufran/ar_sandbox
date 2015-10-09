#include "Terrain.h"


void Terrain::loadTerrain(string filename)
{
    ofBuffer terrainFile = ofBufferFromFile(ofToDataPath(filename));

    cout<<"\nTerrain File:\n"<<terrainFile.getText()<<endl;

    grass = cv::imread(ofToDataPath(terrainFile.getNextLine()),CV_LOAD_IMAGE_COLOR);
    sand = cv::imread(ofToDataPath(terrainFile.getNextLine()),CV_LOAD_IMAGE_COLOR);
    water = cv::imread(ofToDataPath(terrainFile.getNextLine()),CV_LOAD_IMAGE_COLOR);
    pebble = cv::imread(ofToDataPath(terrainFile.getNextLine()),CV_LOAD_IMAGE_COLOR);
    rock_snow = cv::imread(ofToDataPath(terrainFile.getNextLine()),CV_LOAD_IMAGE_COLOR);

    if(!grass.data)            grass=cv::Mat(1440,1920,CV_8UC3,cv::Scalar(0,0,0));
    if(!sand.data)             sand=cv::Mat(1440,1920,CV_8UC3,cv::Scalar(0,0,0));
    if(!water.data)            water=cv::Mat(1440,1920,CV_8UC3,cv::Scalar(0,0,0));
    if(!pebble.data)           pebble=cv::Mat(1440,1920,CV_8UC3,cv::Scalar(0,0,0));
    if(!rock_snow.data)        rock_snow=cv::Mat(1440,1920,CV_8UC3,cv::Scalar(0,0,0));

}
