#include<stdio.h>
#include "ofxCv.h"
#include "ImgProc.h"
#include "ofImage.h"
#include "ofxCvGrayscaleImage.h"
#include "ofxCvColorImage.h"

using namespace ofxCv;
using namespace cv;

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

void boidCollision(Boid b){

//vector<vector<cv::Point>>* contours = ContourFinder.getContours();
/*for (int i =0; i< (*contours)->size(); i++){
        if (*contours)[i].has(b.loc.x, b.loc.y)
                printf("Boid hit");
}*/

}
