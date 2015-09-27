#include<stdio.h>
#include "ofxCv.h"
#include "ImgProc.h"
#include "ofImage.h"
#include "ofxCvGrayscaleImage.h"
#include "ofxCvColorImage.h"
#include <stdio.h>


using namespace ofxCv;
using namespace cv;
void ImgProc:: processImage(){
//img.draw(0,0);
ofxCvGrayscaleImage cvimg;
//cvimg = img;


//gray.updateTexture();
//printf("REgions %d\n",i);
//cout<< "Regions detected "<<regions_detected<<endl;
//ofImage img;
img.loadImage("input.jpg");
cvimg.setFromPixels(img.getPixels(),img.getWidth(),img.getHeight());
//img.setColor(OF_IMAGE_GRAYSCALE);
Mat imgMat = toCv(img);
//img.draw(0,0);
//input = cv::imread("data/input.jpg",1);
//if (!input.data)
 //       printf("Shit happened");
 //contourFinder.setMinAreaRadius(10.0);
 //contourFinder.setMaxAreaRadius(200.0);
contourFinder.setThreshold(230.0);

//contourFinder.findContours(img);
contourFinder.findContours(cvimg);
contourFinder.draw();

}



