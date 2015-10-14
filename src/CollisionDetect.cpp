#include "CollisionDetect.h"

using namespace cv;
using namespace ofxCv;

void CollisionDetect::boidCollision(Boid &b)
{
    for (int i =0; i< contours.size() ; i++)
    {
        // Point polygon test
        double dist = cv::pointPolygonTest(contours[i],cv:: Point(b.loc.x/simRes,b.loc.y/simRes), false);
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

int CollisionDetect::maskCollision(Boid &b,cv::Mat& mask,bool resetPos,bool invert)
{
    float x = b.loc.x/simRes;
    float y = b.loc.y/simRes;

    int compare;

    if(invert)
        compare=255;
    else
        compare=0;

    if(mask.at<uchar>(cv::Point(x,y))==compare)
    {
        if(resetPos)
        {
            if(x<mask.cols/2)
                b.loc.x+=1;
            else
                b.loc.x-=1;

            if(y<mask.rows/2)
                b.loc.y+=1;
            else
                b.loc.y-=1;

            return 2;
        }
        else
        {
            b.collided_with_contour = true;
            b.vel = math::Vec2f(-1,-1)*(b.vel);
            // Setting to Previous frame location
            b.loc =b.prev_loc;

            return 1;
        }


    }
    else
        return 0;

}

void CollisionDetect::boidBBCollision(Boid &b)
{
    for (int i ; i< BBcontours.size(); i++)
    {
        if (BBcontours[i].contains(cv::Point(b.loc.x/simRes,b.loc.y/simRes)))
        {
            cout<<"Boid "<<b.id<<" Hit "<<i<<endl;
            b.collided_with_contour=true;
            b.vel = math::Vec2f(-1,-1)*(b.vel);
            break;
        }
    }
}

void CollisionDetect::nativeContourSetup(float _simRes)
{
    //input=cv::imread("data/fakeKinect.bmp",CV_LOAD_IMAGE_GRAYSCALE);
    //cv::blur(input,input,cv::Size(3,3));
    simRes=_simRes;


}


void CollisionDetect::nativeContourFind()
{
    Mat threshold_output;
    vector<Vec4i> hierarchy;

    cv::threshold( input, threshold_output, thresholdVal, 255, THRESH_BINARY );
    findContours( threshold_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
    //cout<<"Contours After"<<contours.size()<<endl;
}

void CollisionDetect::nativeContourFind(cv::Mat& depthImg)
{

    //outOF.allocate(IMG_WIDTH*res,IMG_HEIGHT*res,OF_IMAGE_COLOR);
    //outOF.setImageType(OF_IMAGE_COLOR_ALPHA);
    Mat threshold_output;
    vector<Vec4i> hierarchy;
    Scalar color = Scalar( 255,255,0 );
    // Find Contours
    cv::threshold( depthImg, threshold_output, thresholdVal, 255, THRESH_BINARY );
    findContours( threshold_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

     /*
    // Place holders for Bounding Box, ellipse
    vector<vector<cv::Point> > contours_poly( contours.size() );
    vector<cv::Rect> boundRect( contours.size() );
    vector<cv::RotatedRect> rotRect(contours.size());


    for( int i = 0; i< contours.size(); i++ )
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
    */
}

void CollisionDetect::nativeDrawContours(int x,int y, int width, int height)
{

    Scalar color = Scalar( 255,0,0 );
    for( int i = 0; i< contours.size(); i++ )
    {
        cv::drawContours( drawing, contours, i, color, 2, 8, vector<Vec4i>(),0, cv::Point() );
    }

    //ofEnableAlphaBlending();
    ofxCv::toOf(drawing,outOF);


    outOF.update();

    //outOF.draw(0,0,WIN_WIDTH*0.5,WIN_HEIGHT*0.5);
    //outOF.draw(WIN_WIDTH*0.2,WIN_HEIGHT*0.5,WIN_WIDTH*0.5,WIN_HEIGHT*0.5);
    outOF.draw(x,y,width,height);
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
