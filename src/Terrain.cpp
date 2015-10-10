#include "Terrain.h"


void Terrain::loadTerrain(string filename,float mapRezImg)
{
    mapRez=mapRezImg;
    ofBuffer terrainFile = ofBufferFromFile(ofToDataPath(filename));

    //cout<<"\nTerrain File:\n"<<terrainFile.getText()<<endl;

    grass = cv::imread(ofToDataPath(terrainFile.getNextLine()),CV_LOAD_IMAGE_COLOR);
    sand = cv::imread(ofToDataPath(terrainFile.getNextLine()),CV_LOAD_IMAGE_COLOR);
    water = cv::imread(ofToDataPath(terrainFile.getNextLine()),CV_LOAD_IMAGE_COLOR);
    pebble = cv::imread(ofToDataPath(terrainFile.getNextLine()),CV_LOAD_IMAGE_COLOR);
    rock_snow = cv::imread(ofToDataPath(terrainFile.getNextLine()),CV_LOAD_IMAGE_COLOR);

    composite=cv::Mat(480*mapRez,640*mapRez,CV_8UC3,cv::Scalar(0,0,0));

    if(!grass.data)            grass=cv::Mat(480*mapRez,640*mapRez,CV_8UC3,cv::Scalar(0,0,0));
    if(!sand.data)             sand=cv::Mat(480*mapRez,640*mapRez,CV_8UC3,cv::Scalar(0,0,0));
    if(!water.data)            water=cv::Mat(480*mapRez,640*mapRez,CV_8UC3,cv::Scalar(0,0,0));
    if(!pebble.data)           pebble=cv::Mat(480*mapRez,640*mapRez,CV_8UC3,cv::Scalar(0,0,0));
    if(!rock_snow.data)        rock_snow=cv::Mat(480*mapRez,640*mapRez,CV_8UC3,cv::Scalar(0,0,0));

    for(int i=0;i<=3;i++)
        mask[i]=cv::Mat(480,640,CV_8UC3,cv::Scalar(0,0,0));

}

void Terrain::setDepthImage(cv::Mat* _depthImage)
{
    depthImage=_depthImage;

}

void Terrain::setThreshold(int threshID, int threshVal)
{
    thresh[threshID]=threshVal;
}

void Terrain::applyThreshold()
{
    cv::threshold(*depthImage,mask[0],thresh[0],255,0);
    for(int i=1;i<3;i++)
    {
        cv::threshold(*depthImage,mask[i],thresh[i],255,3);
        cv::threshold(mask[i],mask[i],thresh[i-1],255,4);
        cv::threshold(mask[i],mask[i],0,255,0);
    }
    cv::threshold(*depthImage,mask[3],thresh[3-1],255,1);
}



void Terrain::multMapWithMask(cv::Mat img, cv::Mat mask, cv::Mat& dest)//very slow function. Don't use.
{
    for(int i=0;i<img.rows;i++)
    {
        for(int j=0;j<img.cols;j++)
        {
            float maskVal = mask.at<uchar>((int)i/mapRez,(int)j/mapRez)/255.0;
            /*if(maskVal==255)
            {
                dest.at<cv::Vec3b>(i,j)[0]=img.at<cv::Vec3b>(i,j)[0];
                dest.at<cv::Vec3b>(i,j)[1]=img.at<cv::Vec3b>(i,j)[1];
                dest.at<cv::Vec3b>(i,j)[2]=img.at<cv::Vec3b>(i,j)[2];
            }*/
            if(maskVal>0)
            {
                dest.at<cv::Vec3b>(i,j)[0]=img.at<cv::Vec3b>(i,j)[0]*maskVal;
                dest.at<cv::Vec3b>(i,j)[1]=img.at<cv::Vec3b>(i,j)[1]*maskVal;
                dest.at<cv::Vec3b>(i,j)[2]=img.at<cv::Vec3b>(i,j)[2]*maskVal;
            }

        }
    }
}

void Terrain::processScene()
{
    applyThreshold();

    //for(int i=0;i<3;i++)
        //cv::GaussianBlur(mask[i],mask[i],cv::Size(11,11),0);

    multMapWithMask(pebble,mask[3],composite);
    multMapWithMask(water,mask[2],composite);
    multMapWithMask(grass,mask[1],composite);
    multMapWithMask(rock_snow,mask[0],composite);

    //combineMapsWithMasks(composite,);



}

cv::Mat* Terrain::getMasks()
{
    return mask;
}

cv::Mat Terrain::getTerrain()
{
    //composite=grass;

    return composite;
}
