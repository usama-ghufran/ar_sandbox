#include "ofApp.h"

using namespace ofxCv;
using namespace cv;

void ofApp::setup()
{
    //INIT variables
    rendererInited = false;
    writeMask=true;
    guiHide = false;
    showContours=false;
    mapRezSim=3;
    mapRezImg=3;
    maskPoints=0;

    // Setup for Collision Detection
    obj.nativeContourSetup(mapRezSim);

    scene.loadTerrain("terrain.txt");
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



    mask=cv::imread("data/mask.bmp",CV_LOAD_IMAGE_GRAYSCALE);
    if(!mask.data)
        mask = cv::Mat(IMG_HEIGHT,IMG_WIDTH,CV_8UC1,cv::Scalar(0));

    scene.loadTerrain("terrain.txt",mapRezImg);
    //flockImg=cv::Mat(IMG_HEIGHT*mapRezImg,IMG_WIDTH*mapRezImg,CV_8UC3,cv::Scalar(0,0,0));
    projectImg=cv::Mat(IMG_HEIGHT*mapRezImg,IMG_WIDTH*mapRezImg,CV_8UC3,cv::Scalar(0,0,0));

    if(fakeKinect)
    {
        depthImage=cv::imread("data/fakeKinect.bmp",CV_LOAD_IMAGE_GRAYSCALE);
        if(!depthImage.data)
            depthImage=cv::Mat(KINECT_HEIGHT,KINECT_WIDTH,CV_8UC1,cv::Scalar(0));
    }
    else
        depthImage=cv::Mat(KINECT_HEIGHT,KINECT_WIDTH,CV_8UC1,cv::Scalar(0));


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

    sim.loadScene(50,50,640*2,480*2,IMG_WIDTH*mapRezSim,IMG_HEIGHT*mapRezSim);
    sim.init(
        100 		,//fish count
        0.1 		,//destination Weight
        0 			,//rand seed
        0.00		,//sleep time
        10 	        ,//boundary padding
        2			,//max speed
        1 			,//max force
        1 	        ,//flock separation weight
        0.5 	    ,//flock alignment weight
        0.25 	    ,//flock cohesion weight
        0 	        ,//collision weight
        15 	        ,//flock separation radius
        20 	        ,//flock alignment radius
        20		    ,//flock cohesion radius
        50		    ,//start position radius
        50            //end position radius
    );

    flockDisplay = sim.getFlockHandle();
    boids = flockDisplay->getBoidsHandle();

    max_height.addListener(this,&ofApp::heightChanged);
    min_height.addListener(this,&ofApp::heightChanged);
    thresh0.addListener(this,&ofApp::thresh0Changed);
    thresh1.addListener(this,&ofApp::thresh1Changed);
    thresh2.addListener(this,&ofApp::thresh2Changed);

    terrainControls.setup("Terrain Controls","terrain_settings.xml",10,10);
    terrainControls.add(max_height.set("Max Height",240,0,255));
    terrainControls.add(min_height.set("Min Height",170,0,255));
    terrainControls.add(thresh0.set("Thresh 0",0.9,0,1));
    terrainControls.add(thresh1.set("Thresh 1",0.75,0,1));
    terrainControls.add(thresh2.set("Thresh 2",0,0,1));
    terrainControls.loadFromFile("terrain_settings.xml");


    showContoursButton.addListener(this,&ofApp::showContoursButtonPressed);
    simControls.setup("Simulation Controls","simulation_settings.xml",10,200);
    simControls.add(showContoursButton.setup("Display Contours"));

    for(int i=0; i<boids->size(); i++)
    {
        while(obj.maskCollision((*boids)[i],mask,true))
        {
            //keep moving the boids till when they are out of mask.
        }
    }
}
void ofApp::heightChanged(int& val)
{
    scene.setThreshold(0,((max_height-min_height)*thresh0)+min_height);
    scene.setThreshold(1,((max_height-min_height)*thresh1)+min_height);
    scene.setThreshold(2,((max_height-min_height)*thresh2)+min_height);
}
void ofApp::thresh0Changed(float& val)
{
    scene.setThreshold(0,((max_height-min_height)*val)+min_height);
}
void ofApp::thresh1Changed(float& val)
{
    scene.setThreshold(1,((max_height-min_height)*val)+min_height);
}
void ofApp::thresh2Changed(float& val)
{
    scene.setThreshold(2,((max_height-min_height)*val)+min_height);
}
void ofApp::showContoursButtonPressed()
{
    showContours=!showContours;

    if(showContours)
            ofSetColor(255,255,255,125);
    else
            ofSetColor(255,255,255,255);
}

void ofApp::update()
{

    depthcam.update();

    if(!fakeKinect)
        depthImage.data=depthcam.getDepthPixels();

    cv::multiply(depthImage,mask,depthImage,1/255.0);

    cv::Scalar whiteC1(255);
    cv::Scalar whiteC3(255,255,255);
    cv::Scalar black(0);
    cv::Scalar green(0,255,0);
    cv::Scalar yellow(255,255,0,125);

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
        sim.frame();

        //Contour Setup
        //obj.processImage();
        obj.nativeContourFind(depthImage);
        //obj.nativeContourFind();

        projectImg = scene.getTerrain();
        cv::cvtColor(projectImg,projectImgRGB,CV_RGB2BGR);



        for(int i=0; i<boids->size(); i++)
        {
            float x = (*boids)[i].loc.x*mapRezImg/mapRezSim;
            float y = (*boids)[i].loc.y*mapRezImg/mapRezSim;

            //Collision Detection and colouring
            obj.boidCollision( (*boids)[i]);
            obj.maskCollision((*boids)[i],mask,false);

            //obj.boidBBCollision( (*boids)[i]);
             if ((*boids)[i].collided_with_contour){
                cv::circle(projectImgRGB,cv::Point(x,y),5,green,-1);
                //cv::circle(flockImg,cv::Point(x,y),3,green,-1);
              // (*boids)[i].collided_with_contour=false;
            }
            else
                cv::circle(projectImgRGB,cv::Point(x,y),5,whiteC3,-1);
                //cv::circle(flockImg,cv::Point(x,y),3,whiteC3,-1);

        }

        //Destination
        cv::circle(projectImgRGB,cv::Point((640*2/mapRezSim)*mapRezImg,(480*2/mapRezSim)*mapRezImg),50,yellow,3);

        ofxCv::toOf(projectImgRGB,projectImgOF);
        projectImgOF.update();
        scene.processScene();
        threshMask = scene.getMasks();

        for(int i =0;i<3;i++)
        {
            ofxCv::toOf(threshMask[i],threshMaskOF[i]);
            threshMaskOF[i].update();
        }


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
        //renderer.drawHueDepthImage();
        //Disable depth test after renderer call.
        ofDisableDepthTest();

        projectImgOF.draw(WIN_WIDTH*0.2,0,WIN_WIDTH*0.5,WIN_HEIGHT*0.5);

        threshMaskOF[0].draw(WIN_WIDTH*0.7,0,WIN_WIDTH*0.3,WIN_HEIGHT*0.3);
        threshMaskOF[1].draw(WIN_WIDTH*0.7,WIN_HEIGHT*0.3,WIN_WIDTH*0.3,WIN_HEIGHT*0.3);
        threshMaskOF[2].draw(WIN_WIDTH*0.7,WIN_HEIGHT*0.6,WIN_WIDTH*0.3,WIN_HEIGHT*0.3);

        ofDrawBitmapString(ofToString(ofGetFrameRate())+" fps", 50, 190);

        if(!guiHide)
        {
            terrainControls.draw();
            simControls.draw();
        }

        if(showContours)
        {
            // Debug to draw contours
            obj.nativeDrawContours(WIN_WIDTH*0.2,0,WIN_WIDTH*0.5,WIN_HEIGHT*0.5);
        }


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
        break;
    case 't':
        obj.thresholdVal-=1;
        break;
    case 'h':
        guiHide=!guiHide;
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
