#include<stdio.h>
#include "ofxCv.h"
#include "ImgProc.h"
#include "ofImage.h"
#include "ofxCvGrayscaleImage.h"
#include "ofxCvColorImage.h"
#include "Vector.h"

using namespace cv;
using namespace ofxCv;


void ImgProc:: processImage(){

//cvimg = img;
//gray.updateTexture();
//printf("REgions %d\n",i);
//cout<< "Regions detected "<<regions_detected<<endl;
//ofImage img;

/*img.loadImage("input.jpg");
cvimg.setFromPixels(img.getPixels(),img.getWidth(),img.getHeight());
Mat imgMat = toCv(img);*/
//img.setColor(OF_IMAGE_GRAYSCALE);

//cvimg.draw(0,0);
//input = cv::imread("data/input.jpg",1);
//if (!input.data)
 //       printf("Shit happened");
// contourFinder.setMinAreaRadius(10.0);
 //contourFinder.setMaxAreaRadius(200.0);
contourFinder.setThreshold(thresholdVal);
vector<vector<cv::Point> > contours = contourFinder.getContours();
//contourFinder.findContours(img);
contourFinder.findContours(cvimg);
}
void ImgProc::boidBBCollision(Boid &b){


}

void ImgProc::boidCollision(Boid &b){

//vector<vector<cv::Point> > contours = contourFinder.getContours();
//cout<<"Contours Size"<<contours.size();

for (int i =0; i< contours.size() ; i++){

        vector<cv::Point>::iterator it = std::find(contours[i].begin(), contours[i].end(),cv::Point(b.loc.x, b.loc.y));
        if(it!=contours[i].end()){
                printf("Boid %d hit  contour %d of size %d\n", b.id,i,contours[i].size());
                b.collided_with_contour = true;
                b.vel = math::Vec2f(-1,-1)*(b.vel);
        }
}
}

void ImgProc::nativeContourSetup(){

input=cv::imread("data/fakeKinect.bmp",CV_LOAD_IMAGE_GRAYSCALE);
cv::blur(input,input,cv::Size(3,3));
}

void ImgProc::nativeContourFind(){
Mat threshold_output;
vector<Vec4i> hierarchy;

cv::threshold( input, threshold_output, thresholdVal, 255, THRESH_BINARY );
//cv::Canny( input, threshold_output, thresholdVal, thresholdVal*2, 3 );
//cout<<"Threshold "<<thresholdVal;

findContours( threshold_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
cout<<"Contours After"<<contours.size()<<endl;
}

void ImgProc::nativeContourFind(cv::Mat depthImg){
Mat threshold_output;
vector<Vec4i> hierarchy;


cv::threshold( depthImg, threshold_output, thresholdVal, 255, THRESH_BINARY );
findContours( threshold_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
vector<vector<cv::Point> > contours_poly( contours.size() );
vector<cv::Rect> boundRect( contours.size() );
Scalar color = Scalar( 255,255,0 );

//cv::Mat drawing=cv::Mat(480,640,CV_8UC3,cv::Scalar(0,0,0));
for( int i = 0; i< contours.size()        ; i++ ){
approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
boundRect[i] = boundingRect( Mat(contours_poly[i]) );
BBcontours.push_back(boundRect[i]);
 cv::rectangle( drawing, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0 );
}
}

void ImgProc::nativeDrawContours(){

Scalar color = Scalar( 255,0,0 );
 // for( int i = 0; i< 1        ; i++ ){
for( int i = 0; i< contours.size()        ; i++ ){
       //cv::drawContours( drawing, contours, i, color, 2, 8, vector<Vec4i>(),0, cv::Point() );
}

//cout<<"Threshold "<<thresholdVal;
//cout<<"Contours After"<<contours.size()<<endl;
ofImage outOF;
ofEnableAlphaBlending();
ofxCv::toOf(drawing,outOF);

//drawing =cv::Mat(480,640,CV_8UC3,cv::Scalar(0,0,0));
outOF.setImageType(OF_IMAGE_COLOR_ALPHA);
ofSetColor(255,255,255,127);
outOF.update();
//outOF.draw(WIN_WIDTH*0.5,0,WIN_WIDTH*0.5,WIN_HEIGHT*0.5);
outOF.draw(0,0,640/2.0*2,480/2.0*2);
drawing = Scalar(0,0,0);
//cv::Mat drawing=cv::Mat(480,640,CV_8UC3,cv::Scalar(0,0,0));
}
