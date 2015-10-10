#include "ofApp.h"

using namespace ofxCv;
using namespace cv;

void ofApp::setup()
{

    // Setup for Collision Detection
    obj.nativeContourSetup();
    scene.loadTerrain("terrain.txt");

    arguments = ofxArgParser::allKeys();
    ofSetLogLevel(OF_LOG_WARNING);
    ofSetVerticalSync(false);

    depthcam.setRegistration(true);
    depthcam.init();
    depthcam.setUseTexture(false);
    fakeKinect=!(depthcam.open());
    dataset.loadFile("calibrationData.xml");

    calibration.init(&depthcam,&dataset);
    calibration.enableKeys();
    calibration.enableChessboardMouseControl();

    thresh=210;
    rendererInited = false;
    writeMask=true;
    mapRezSim=3;
    mapRezImg=3;
    maskPoints=0;

    mask=cv::imread("data/mask.bmp",1);

    if(fakeKinect)
    {
        depthImage=cv::imread("data/fakeKinect.bmp",CV_LOAD_IMAGE_GRAYSCALE);
        if(!depthImage.data)
            depthImage=cv::Mat(KINECT_HEIGHT,KINECT_WIDTH,CV_8UC1,cv::Scalar(0));
    }
    else
        depthImage=cv::Mat(KINECT_HEIGHT,KINECT_WIDTH,CV_8UC1,cv::Scalar(0));

    //depthImageThresh=cv::Mat(480,640,CV_8UC1,cv::Scalar(0));
    flockImg=cv::Mat(IMG_HEIGHT*mapRezImg,IMG_WIDTH*mapRezImg,CV_8UC3,cv::Scalar(0,0,0));
    flockingImgOF.allocate(IMG_WIDTH*mapRezImg,IMG_HEIGHT*mapRezImg,OF_IMAGE_COLOR);
    depthView.allocate(KINECT_WIDTH,KINECT_HEIGHT,OF_IMAGE_GRAYSCALE);

    if(!mask.data)
        mask = cv::Mat(IMG_HEIGHT,IMG_WIDTH,CV_8UC1,cv::Scalar(0));

    //Parse Args
    for (int i = 0; i < arguments.size(); i++)
    {
        if(arguments[i]=="calib")
        {
            doCalib = ofToInt(ofxArgParser::getValue(arguments[i]));
        }
    }

    //Decide if to use the existing calibration and bypass the calibration step
    if(doCalib==0)
    {
        calibration.update();
        calibration.finalize();
    }

    /*
        sim.init(
            mfishCount 			,
            mdestWeight 		,
            mrandSeed 			,
            msleepTime			,
            mboundaryPadding 	,
            mmaxSpeed 			,
            mmaxForce 			,
            mflockSepWeight 	,
            mflockAliWeight 	,
            mflockCohWeight 	,
            mcollisionWeight 	,
            mflockSepRadius 	,
            mflockAliRadius 	,
            mflockCohRadius		,
            mstartPosRad		,
            mendPosRad
    		);
    		*/
    sim.loadScene(50,50,100,100,IMG_WIDTH*mapRezSim,IMG_HEIGHT*mapRezSim);
    sim.init(
        100 			,
        0.01 		,
        0 			,
        0.02			,
        10 	,
        2			,
        1 			,
        1 	,
        0.5 	,
        0.25 	,
        0 	,
        15 	,
        20 	,
        20		,
        50		,
        0
    );

    flockDisplay = sim.getFlockHandle();
}

void ofApp::update()
{

    depthcam.update();

    if(!fakeKinect)
        depthImage.data=depthcam.getDepthPixels();

    cv::Scalar whiteC1(255);
    cv::Scalar whiteC3(255,255,255);
    cv::Scalar black(0);
    cv::Scalar green(0,255,0);


    if(!calibration.isFinalized())
    {
        calibration.update();
        if(maskPoints==4 && writeMask)
        {
            mask = black;
            cv::fillConvexPoly(mask,kinectMask,4,whiteC1);
            cv::imwrite("data/mask.bmp",mask);
            writeMask=false;
        }
    }

    if(calibration.isFinalized() && !rendererInited)
    {
        renderer.init(&depthcam);
        renderer.setDrawArea(WIN_WIDTH,0,WIN_WIDTH,WIN_HEIGHT);
        renderer.setProjectionMatrix(dataset.getMatrix());
        rendererInited = true;
    }

    if(calibration.isFinalized() && rendererInited)
    {
        renderer.update();
        sim.frame();
        //Contour Setup
        //obj.processImage();
        obj.nativeContourFind(depthImage);
        //obj.nativeContourFind();


        vector<Boid>* boids = flockDisplay->getBoidsHandle();
        flockImg=black;

        for(int i=0; i<boids->size(); i++)
        {
            float x = (*boids)[i].loc.x*mapRezImg/mapRezSim;
            float y = (*boids)[i].loc.y*mapRezImg/mapRezSim;
            //Collision Detection and colouring
            obj.boidCollision( (*boids)[i]);
            //obj.boidBBCollision( (*boids)[i]);
             if ((*boids)[i].collided_with_contour){
                cv::circle(flockImg,cv::Point(x,y),3,green,-1);
              // (*boids)[i].collided_with_contour=false;
            }
            else
                cv::circle(flockImg,cv::Point(x,y),3,whiteC3,-1);
        }

        ofxCv::toOf(flockImg,flockingImgOF);
        flockingImgOF.update();

        cv::threshold(depthImage,depthImageThresh,thresh,255,0);
        ofxCv::toOf(depthImageThresh,depthView);
        depthView.update();

    }
}

void ofApp::draw()
{
    if(!calibration.isFinalized())
    {
        calibration.drawStatusScreen(0,0,WIN_WIDTH,WIN_HEIGHT);
        calibration.drawChessboard(WIN_WIDTH,0,WIN_WIDTH,WIN_HEIGHT);

        //Draw Mask Points;
        ofSetColor(255,0,0);

        for(int i=0; i<maskPoints; i++)
        {
            ofCircle(drawkinectMask[i].x,drawkinectMask[i].y,5);
            ofDrawBitmapString(ofToString(i),drawkinectMask[i].x,drawkinectMask[i].y);
        }
        ofSetColor(255,255,255);
    }

    if(calibration.isFinalized() && rendererInited)
    {
        //renderer.drawHueDepthImage();
        flockingImgOF.draw(0,0,WIN_WIDTH*0.5,WIN_HEIGHT*0.5);
        depthView.draw(WIN_WIDTH*0.5,0,WIN_WIDTH*0.5,WIN_HEIGHT*0.5);
        // Debug to draw contours
        obj.nativeDrawContours();
    }

}

void ofApp::keyPressed(int key)
{

    switch (key)
    {
    case OF_KEY_ESC:
        maskPoints=0;
        break;
    case 'K':
        imwrite("data/fakeKinect.bmp",depthImage);
        break;
    case 'T':
        obj.thresholdVal+=1;
        thresh+=1;
        cout<<"\nthresh "<<thresh<<endl;
        break;
    case 't':
        obj.thresholdVal-=1;
        thresh-=1;
        cout<<"\nthresh "<<thresh<<endl;
        break;
    }
}

void ofApp::mousePressed(int x, int y, int button)
{

    if(!calibration.isFinalized())
    {
        //cout<<"\nMOUSE: x y button "<<x<<" "<<y<<" "<<button;

        if(maskPoints>=4)
        {
            maskPoints=0;
            writeMask=true;
        }
        kinectMask[maskPoints]=cv::Point(x*2*IMG_WIDTH/(WIN_WIDTH),y*2*IMG_HEIGHT/(WIN_HEIGHT));
        drawkinectMask[maskPoints]=cv::Point(x,y);
        maskPoints++;
    }
}

void ofApp::exit()
{
}
