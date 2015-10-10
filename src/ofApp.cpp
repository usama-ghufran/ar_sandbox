#include "ofApp.h"

using namespace ofxCv;
using namespace cv;


void ofApp::setup()
{

    arguments = ofxArgParser::allKeys();
    ofSetLogLevel(OF_LOG_ERROR);
    ofSetVerticalSync(false);

    depthcam.setRegistration(true);
    depthcam.init();
    depthcam.setUseTexture(false);
    fakeKinect=!(depthcam.open());
    dataset.loadFile("calibrationData.xml");

    calibration.init(&depthcam,&dataset);
    calibration.enableKeys();
    calibration.enableChessboardMouseControl();

    rendererInited = false;
    writeMask=true;
    mapRezSim=3;
    mapRezImg=3;
    maskPoints=0;

    mask=cv::imread("data/mask.bmp",1);
    scene.loadTerrain("terrain.txt",mapRezImg);

    if(fakeKinect)
    {
        depthImage=cv::imread("data/fakeKinect.bmp",CV_LOAD_IMAGE_GRAYSCALE);
        if(!depthImage.data)
            depthImage=cv::Mat(KINECT_HEIGHT,KINECT_WIDTH,CV_8UC1,cv::Scalar(0));
    }
    else
        depthImage=cv::Mat(KINECT_HEIGHT,KINECT_WIDTH,CV_8UC1,cv::Scalar(0));

    flockImg=cv::Mat(IMG_HEIGHT*mapRezImg,IMG_WIDTH*mapRezImg,CV_8UC3,cv::Scalar(0,0,0));

    //flockingImgOF.allocate(IMG_WIDTH*mapRezImg,IMG_HEIGHT*mapRezImg,OF_IMAGE_COLOR);

    projectImg=cv::Mat(IMG_HEIGHT*mapRezImg,IMG_WIDTH*mapRezImg,CV_8UC3,cv::Scalar(0,0,0));

    //depthView.allocate(KINECT_WIDTH,KINECT_HEIGHT,OF_IMAGE_GRAYSCALE);

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
    scene.setThreshold(0,230);
    scene.setThreshold(1,220);
    scene.setThreshold(2,210);
}

void ofApp::update()
{

    depthcam.update();

    if(!fakeKinect)
        depthImage.data=depthcam.getDepthPixels();


    cv::Scalar whiteC1(255);
    cv::Scalar whiteC3(255,255,255);
    cv::Scalar black(0);

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
        scene.setDepthImage(&depthImage);
    }

    if(calibration.isFinalized() && rendererInited)
    {
        renderer.update();
        //sim.frame();

        vector<Boid>* boids = flockDisplay->getBoidsHandle();
        flockImg=black;

        for(int i=0; i<boids->size(); i++)
        {
            float x = (*boids)[i].loc.x*mapRezImg/mapRezSim;
            float y = (*boids)[i].loc.y*mapRezImg/mapRezSim;
            cv::circle(flockImg,cv::Point(x,y),3,whiteC3,-1);
        
            /* Collision Detection Place holder
            obj.boidCollision( (*boids)[i]);
            //obj.boidBBCollision( (*boids)[i]);
             if ((*boids)[i].collided_with_contour){
                cv::circle(flockImg,cv::Point(x,y),3,green,-1);
              // (*boids)[i].collided_with_contour=false;
            }
            else*/

        }

        //projectImg = scene.getTerrain();
        //cv::cvtColor(projectImg,projectImgRGB,CV_RGB2BGR);
        //ofxCv::toOf(projectImgRGB,projectImgOF);
        //projectImgOF.update();
        scene.processScene();
        threshMask = scene.getMasks();

        for(int i =0;i<3;i++)
        {
            ofxCv::toOf(threshMask[i],threshMaskOF[i]);
            threshMaskOF[i].update();
        }

        ofxCv::toOf(flockImg,flockingImgOF);
        flockingImgOF.update();

        ofxCv::toOf(depthImage,depthImageOF);
        depthImageOF.update();


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
        renderer.drawHueDepthImage();
        //flockingImgOF.draw(0,0,WIN_WIDTH*0.5,WIN_HEIGHT*0.5);
        //depthImageOF.draw(WIN_WIDTH*0.5,0,WIN_WIDTH*0.5,WIN_HEIGHT*0.5);
        //projectImgOF.draw(0,WIN_HEIGHT*0.5,WIN_WIDTH*0.5,WIN_HEIGHT*0.5);

        threshMaskOF[0].draw(0,0,WIN_WIDTH*0.5,WIN_HEIGHT*0.5);
        threshMaskOF[1].draw(WIN_WIDTH*0.5,0,WIN_WIDTH*0.5,WIN_HEIGHT*0.5);
        threshMaskOF[2].draw(0,WIN_HEIGHT*0.5,WIN_WIDTH*0.5,WIN_HEIGHT*0.5);

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
