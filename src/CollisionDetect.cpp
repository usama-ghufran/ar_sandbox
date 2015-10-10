#include<stdio.h>
#include "ofxCv.h"
#include "CollisionDetect.h"
#include "ofImage.h"
#include "ofxCvGrayscaleImage.h"
#include "ofxCvColorImage.h"
#include "Vector.h"

using namespace cv;
using namespace ofxCv;

void CollisionDetect::boidCollision(Boid &b)
{
    for (int i =0; i< contours.size() ; i++)
    {
        // Point polygon test
        double dist = cv::pointPolygonTest(contours[i],cv:: Point(b.loc.x,b.loc.y), false);
        int offset = 5;
        if (dist >= 0)
        {
            //cout << "Boid " << b.id<<"Hit Contour "<< i<<endl;
            b.collided_with_contour = true;
            b.vel = math::Vec2f(-1,-1)*(b.vel);
            // Setting to Previous frame location
            b.loc =b.prev_loc;
        }
    }
}

void CollisionDetect::boidBBCollision(Boid &b)
{
    for (int i ; i< BBcontours.size(); i++)
    {
        if (BBcontours[i].contains(cv::Point(b.loc.x,b.loc.y)))
        {
            cout<<"Boid "<<b.id<<" Hit "<<i<<endl;
            b.collided_with_contour=true;
            b.vel = math::Vec2f(-1,-1)*(b.vel);
            break;
        }
    }
}

void CollisionDetect::nativeContourSetup()
{
    input=cv::imread("data/fakeKinect.bmp",CV_LOAD_IMAGE_GRAYSCALE);
    cv::blur(input,input,cv::Size(3,3));
}


void CollisionDetect::nativeContourFind()
{
    Mat threshold_output;
    vector<Vec4i> hierarchy;

    cv::threshold( input, threshold_output, thresholdVal, 255, THRESH_BINARY );
    findContours( threshold_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
    //cout<<"Contours After"<<contours.size()<<endl;
}

void CollisionDetect::nativeContourFind(cv::Mat depthImg)
{
    Mat threshold_output;
    vector<Vec4i> hierarchy;
    Scalar color = Scalar( 255,255,0 );
    // Find Contours
    cv::threshold( depthImg, threshold_output, thresholdVal, 255, THRESH_BINARY );
    findContours( threshold_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

    // Place holders for Bounding Box, ellipse
    vector<vector<cv::Point> > contours_poly( contours.size() );
    vector<cv::Rect> boundRect( contours.size() );
    vector<cv::RotatedRect> rotRect(contours.size());

    for( int i = 0; i< contours.size()        ; i++ )
    {
        approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
        boundRect[i] = boundingRect( Mat(contours_poly[i]) );
        BBcontours.push_back(boundRect[i]);

        if (contours[i].size()>5)
        {
            rotRect[i] = cv::fitEllipse(Mat (contours[i]));
            ellipse(drawing,rotRect[i],color, 2,8);
        }
        //cv::rectangle( drawing, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0 );
    }
}

void CollisionDetect::nativeDrawContours()
{

    Scalar color = Scalar( 255,0,0 );
    for( int i = 0; i< contours.size(); i++ )
    {
        cv::drawContours( drawing, contours, i, color, 2, 8, vector<Vec4i>(),0, cv::Point() );
    }
    ofImage outOF;
    outOF.allocate(IMG_WIDTH*res,IMG_HEIGHT*res,OF_IMAGE_COLOR);
    ofEnableAlphaBlending();
    ofxCv::toOf(drawing,outOF);
    outOF.setImageType(OF_IMAGE_COLOR_ALPHA);
    ofSetColor(255,255,255,150);
    outOF.update();

    //outOF.draw(0,0,WIN_WIDTH*0.5,WIN_HEIGHT*0.5);
    outOF.draw(WIN_WIDTH*0.2,0,WIN_WIDTH*0.5,WIN_HEIGHT*0.5);
    drawing = Scalar(0,0,0);
    BBcontours.clear();

}

// Module to use  ofxCV contour detection
void CollisionDetect:: processImage()
{
    contourFinder.setThreshold(thresholdVal);
    vector<vector<cv::Point> > contours = contourFinder.getContours();
    contourFinder.findContours(cvimg);
}
