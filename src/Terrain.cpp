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
    brick = cv::imread(ofToDataPath(terrainFile.getNextLine()),CV_LOAD_IMAGE_COLOR);

    composite=cv::Mat(480*mapRez,640*mapRez,CV_8UC3,cv::Scalar(0,0,0));

    if(!grass.data)            grass=cv::Mat(480*mapRez,640*mapRez,CV_8UC3,cv::Scalar(0,0,0));
    if(!sand.data)             sand=cv::Mat(480*mapRez,640*mapRez,CV_8UC3,cv::Scalar(0,0,0));
    //if(!water.data)
                water=cv::Mat(480*mapRez,640*mapRez,CV_8UC3,cv::Scalar(0,0,0));
    //if(!pebble.data)
               pebble=cv::Mat(480*mapRez,640*mapRez,CV_8UC3,cv::Scalar(0,0,0));
    if(!rock_snow.data)        rock_snow=cv::Mat(480*mapRez,640*mapRez,CV_8UC3,cv::Scalar(0,0,0));
    if(!brick.data)        rock_snow=cv::Mat(480*mapRez,640*mapRez,CV_8UC3,cv::Scalar(0,0,0));

    for(int i=0;i<=3;i++)
        mask[i]=cv::Mat(480,640,CV_8UC1,cv::Scalar(0));

    //textures[0]=&rock_snow;
    textures[0]=&grass;
    textures[1]=&brick;
    textures[2]=&water;
    textures[3]=&pebble;
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
    for(int i=1;i<3-1;i++)
    {
        cv::threshold(*depthImage,mask[i],thresh[i],255,3);
        cv::threshold(mask[i],mask[i],thresh[i-1],255,4);
        cv::threshold(mask[i],mask[i],0,255,0);
    }
    cv::threshold(*depthImage,mask[3-1],thresh[3-2],255,1);
}

void Terrain::combineMapsWithMasks()
{
    int mres = (int)mapRez;
    for(int i = 0; i < composite.rows; i++)
    {
        cv::Vec3b* comp = composite.ptr<cv::Vec3b>(i);

        cv::Vec3b* t0 = textures[0]->ptr<cv::Vec3b>(i);
        cv::Vec3b* t1 = textures[1]->ptr<cv::Vec3b>(i);
        cv::Vec3b* t2 = textures[2]->ptr<cv::Vec3b>(i);
        cv::Vec3b* t3 = textures[3]->ptr<cv::Vec3b>(i);

        uchar* m0 = mask[0].ptr<uchar>(i/mres);
        uchar* m1 = mask[1].ptr<uchar>(i/mres);
        uchar* m2 = mask[2].ptr<uchar>(i/mres);
        uchar* m3 = mask[3].ptr<uchar>(i/mres);


        for(int j = 0; j < composite.cols; j++)
        {
            if(m3[j/mres]==255)
                comp[j]=t3[j];

            else if(m2[j/mres]==255)
                comp[j]=t2[j];

            else if(m1[j/mres]==255)
                comp[j]=t1[j];

            else if(m0[j/mres]==255)
                comp[j]=t0[j];
        }
    }
}


void Terrain::combineMapsWithMasks2()//very slow function. Don't use.
{
    cv::MatIterator_<cv::Vec3b> itComp = composite.begin<cv::Vec3b>(), it_endComp = composite.end<cv::Vec3b>();

    cv::MatConstIterator_<cv::Vec3b> it0 = textures[0]->begin<cv::Vec3b>(), it_end0 = textures[0]->end<cv::Vec3b>();
    cv::MatConstIterator_<cv::Vec3b> it1 = textures[1]->begin<cv::Vec3b>(), it_end1 = textures[0]->end<cv::Vec3b>();
    cv::MatConstIterator_<cv::Vec3b> it2 = textures[2]->begin<cv::Vec3b>(), it_end2 = textures[0]->end<cv::Vec3b>();
    cv::MatConstIterator_<cv::Vec3b> it3 = textures[3]->begin<cv::Vec3b>(), it_end3 = textures[0]->end<cv::Vec3b>();


    cv::MatConstIterator_<uchar> itM0 = mask[0].begin<uchar>(), it_endM0 = mask[0].end<uchar>();
    cv::MatConstIterator_<uchar> itM1 = mask[1].begin<uchar>(), it_endM1 = mask[1].end<uchar>();
    cv::MatConstIterator_<uchar> itM2 = mask[2].begin<uchar>(), it_endM2 = mask[2].end<uchar>();
    cv::MatConstIterator_<uchar> itM3 = mask[3].begin<uchar>(), it_endM3 = mask[3].end<uchar>();

    for(int maskIterator=0; itComp != it_endComp; ++itComp,maskIterator++)
    {
        float maskVal0 = (*itM0)/255.0;
        float maskVal1 = (*itM1)/255.0;
        float maskVal2 = (*itM2)/255.0;
        float maskVal3 = (*itM3)/255.0;

        (*itComp)[0]=0;
        (*itComp)[1]=0;
        (*itComp)[2]=0;
        if(maskVal0>0)
        {
            (*itComp)[0]+= (((*it0)[0]) * (maskVal0));
            (*itComp)[1]+= (((*it0)[1]) * (maskVal0));
            (*itComp)[2]+= (((*it0)[2]) * (maskVal0));
        }


        if(maskVal1>0)
        {
            (*itComp)[0]+= (((*it1)[0]) * (maskVal1));
            (*itComp)[1]+= (((*it1)[1]) * (maskVal1));
            (*itComp)[2]+= (((*it1)[2]) * (maskVal1));
        }


        if(maskVal2>0)
        {
            (*itComp)[0]+= (((*it2)[0]) * (maskVal2));
            (*itComp)[1]+= (((*it2)[1]) * (maskVal2));
            (*itComp)[2]+= (((*it2)[2]) * (maskVal2));
        }


        if(maskVal3>0)
        {
            (*itComp)[0]+= (((*it3)[0]) * (maskVal3));
            (*itComp)[1]+= (((*it3)[1]) * (maskVal3));
            (*itComp)[2]+= (((*it3)[2]) * (maskVal3));
        }


        if(maskIterator%((int)mapRez)==(int)(mapRez-1))
        {
            itM0++;
            itM1++;
            itM2++;
            itM3++;
        }

        it0++;
        it1++;
        it2++;
        it3++;




    }


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
        //cv::GaussianBlur(mask[1],mask[1],cv::Size(5,5),0);

    //multMapWithMask(pebble,mask[3],composite);
    //multMapWithMask(water,mask[2],composite);
    //multMapWithMask(grass,mask[1],composite);
    //multMapWithMask(rock_snow,mask[0],composite);

    //combineMapsWithMasks2();


    combineMapsWithMasks();



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

void Terrain::translateImg(cv::Mat &img,cv::Mat& dst, int offsetx, int offsety)
{
    cv::Mat trans_mat = (cv::Mat_<double>(2,3) << 1, 0, offsetx, 0, 1, offsety);
    cv::warpAffine(img,dst,trans_mat,img.size());

}
