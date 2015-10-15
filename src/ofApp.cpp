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
    mapRezImg=1;
    maskPoints=0;
    fishCount=100;
    boundaryPadding=10;
    collisionWeight=0;
    startPosx=startPosy=100;
    endPosx=endPosy=IMG_HEIGHT*mapRezSim*0.8;

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
    projectImgRGBMoved=cv::Mat(IMG_HEIGHT*mapRezImg,IMG_WIDTH*mapRezImg,CV_8UC3,cv::Scalar(0,0,0));

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
    boidScale.addListener(this,&ofApp::boidTriangleScaleChanged);
    simControls.setup("Simulation Controls","simulation_settings.xml",10,200);
    simControls.add(showContoursButton.setup("Display Contours"));
    simControls.add(maxSpeed.set("Max Speed",2,0,10));
    simControls.add(maxForce.set("Max Force",1,0,10));
    simControls.add(destWeight.set("Goal Weight",0.1,0,1));
    simControls.add(flockSeparationWeight.set("Separation Weight",1,0,2));
    simControls.add(flockAlignmentWeight.set("Alignment Weight",0.5,0,2));
    simControls.add(flockCohesionWeight.set("Cohesion Weight",0.25,0,2));
    simControls.add(flockSeparationRadius.set("Separation Radius",15,0,50));
    simControls.add(flockAlignmentRadius.set("Alignment Radius",20,0,50));
    simControls.add(flockCohesionRadius.set("Cohesion Radius",20,0,50));
    simControls.add(startRadius.set("Start Radius",50,0,200));
    simControls.add(endRadius.set("End Radius",50,0,200));
    simControls.add(sleepTime.set("Sim Sleep Time",0,0,0.1));
    simControls.add(randSeed.set("Random Seed",0,0,1));
    simControls.add(boidScale.set("Boid Scale",2,0,4));
    simControls.loadFromFile("simulation_settings.xml");


    destWeight.addListener(this,&ofApp::simParamChanged);
    randSeed.addListener(this,&ofApp::simParamChanged);
    sleepTime.addListener(this,&ofApp::simParamChanged);
    maxSpeed.addListener(this,&ofApp::simParamChanged);
    maxForce.addListener(this,&ofApp::simParamChanged);
    flockSeparationWeight.addListener(this,&ofApp::simParamChanged);
    flockAlignmentWeight.addListener(this,&ofApp::simParamChanged);
    flockCohesionWeight.addListener(this,&ofApp::simParamChanged);
    flockSeparationRadius.addListener(this,&ofApp::simParamChanged);
    flockAlignmentRadius.addListener(this,&ofApp::simParamChanged);
    flockCohesionRadius.addListener(this,&ofApp::simParamChanged);
    startRadius.addListener(this,&ofApp::simParamStartRadiusChanged);
    endRadius.addListener(this,&ofApp::simParamEndRadiusChanged);

    sim.loadScene(startPosx,startPosy,endPosx,endPosy,IMG_WIDTH*mapRezSim,IMG_HEIGHT*mapRezSim);
    sim.init(
        fishCount 		            ,//fish count
        destWeight 		            ,//destination Weight
        randSeed 		            ,//rand seed
        sleepTime		            ,//sleep time
        boundaryPadding 	        ,//boundary padding
        maxSpeed			        ,//max speed
        maxForce 			        ,//max force
        flockSeparationWeight 	    ,//flock separation weight
        flockAlignmentWeight 	    ,//flock alignment weight
        flockCohesionWeight 	    ,//flock cohesion weight
        collisionWeight 	        ,//collision weight
        flockSeparationRadius 	    ,//flock separation radius
        flockAlignmentRadius 	    ,//flock alignment radius
        flockCohesionRadius		    ,//flock cohesion radius
        startRadius		            ,//start position radius
        endRadius                    //end position radius
    );

    flockDisplay = sim.getFlockHandle();
    boids = flockDisplay->getBoidsHandle();


    for(int i=0; i<boids->size(); i++)
    {
        while(obj.maskCollision((*boids)[i],mask,true)==2)
        {
            //keep moving the boids till when they are out of mask.
        }
    }


    imageControls.setup("Image Controls","simulation_settings.xml",10,600);
    imageControls.add(offsetx.set("Offset X",0,-100,100));
    imageControls.add(offsety.set("Offset Y",0,-100,100));


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
void ofApp::simParamChanged(float& val)
{
    sim.updateSimParams(
        destWeight 		            ,//destination Weight
        randSeed 		            ,//rand seed
        sleepTime		            ,//sleep time
        boundaryPadding 	        ,//boundary padding
        maxSpeed			        ,//max speed
        maxForce 			        ,//max force
        flockSeparationWeight 	    ,//flock separation weight
        flockAlignmentWeight 	    ,//flock alignment weight
        flockCohesionWeight 	    ,//flock cohesion weight
        collisionWeight 	            ,//collision weight
        flockSeparationRadius 	    ,//flock separation radius
        flockAlignmentRadius 	    ,//flock alignment radius
        flockCohesionRadius		    ,//flock cohesion radius
        startRadius		            ,//start position radius
        endRadius                   ); //end position radius);
}
void ofApp::simParamStartRadiusChanged(float& val)
{
    sim.setStart(startPosx,startPosy,startRadius);
}
void ofApp::simParamEndRadiusChanged(float& val)
{
    sim.setDestination(endPosx,endPosy,endRadius);
}
void ofApp::boidTriangleScaleChanged(float& val)
{
    trianglePts[0] = cv::Point(val*2.5-val*0.5,0);
    trianglePts[1] = cv::Point(-val-val*0.5,-val);
    trianglePts[2] = cv::Point(-val-val*0.5,val);
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
    cv::Scalar yellow(255,255,0);
    cv::Scalar blue(0,0,255);
    cv::Scalar cyan(0,255,255);

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

    else if(calibration.isFinalized() && !rendererInited)
    {
        renderer.init(&depthcam);
        renderer.setDrawArea(WIN_WIDTH,0,WIN_WIDTH,WIN_HEIGHT);
        renderer.setProjectionMatrix(dataset.getMatrix());
        rendererInited = true;

        scene.setDepthImage(&depthImage);
    }

    else if(calibration.isFinalized() && rendererInited)
    {
        renderer.update();


        if(!sim.frame())
        {
            cout<<"\nAdding new boids\n";
            sim.addAllBoids();
        }

        obj.nativeContourFind(depthImage);

        projectImg = scene.getTerrain();
        cv::cvtColor(projectImg,projectImgRGB,CV_RGB2BGR);
        threshMask = scene.getMasks();
        for(int i=0; i<boids->size(); i++)
        {
            float x = (*boids)[i].loc.x*mapRezImg/mapRezSim;
            float y = (*boids)[i].loc.y*mapRezImg/mapRezSim;

            //Collision Detection and colouring
            //obj.boidCollision( (*boids)[i]);
            if(obj.maskCollision((*boids)[i],mask,false)==1)
            {
                if(obj.maskCollision((*boids)[i],mask,false)!=0)
                {
                    while(obj.maskCollision((*boids)[i],mask,true)==2)
                    {
                        //keep moving the boids till when they are out of mask.
                    }
                }
            }
            //cv::add(threshMask[0],threshMask[1],threshMask[1]);//slow
            //obj.maskCollision((*boids)[i],threshMask[1],false,true);
            obj.maskCollision((*boids)[i],threshMask[2],false);
            cv::Point boidPts[3];
            float angle=(*boids)[i].orient/180.0*PI;

            for(int i=0;i<3;i++)
            {
                boidPts[i].x=trianglePts[i].x*cos(angle)-trianglePts[i].y*sin(angle);
                boidPts[i].y=trianglePts[i].y*cos(angle)+trianglePts[i].x*sin(angle);
                boidPts[i]+=cv::Point(x,y);
            }

            //obj.boidBBCollision( (*boids)[i]);
             //if ((*boids)[i].collided_with_contour){
                //cv::circle(projectImgRGB,cv::Point(x,y),5,green,-1);


               // cv::fillConvexPoly(projectImgRGB,boidPts,3,green);
                //cv::circle(flockImg,cv::Point(x,y),3,green,-1);
              // (*boids)[i].collided_with_contour=false;
            //}
            //else
                //cv::circle(projectImgRGB,cv::Point(x,y),5,whiteC3,-1);
                //cv::fillConvexPoly(projectImgRGB,boidPts,3,cyan);
                cv::Scalar randomCol(randomRange(50,255,(*boids)[i].id*123),randomRange(50,255,(*boids)[i].id*157),randomRange(50,255,(*boids)[i].id*921));
                cv::fillConvexPoly(projectImgRGB,boidPts,3,randomCol);

        }

        //Start
        cv::circle(projectImgRGB,cv::Point((startPosx/mapRezSim)*mapRezImg,(startPosy/mapRezSim)*mapRezImg),(startRadius/mapRezSim)*mapRezImg,blue,1);
        //Destination
        cv::circle(projectImgRGB,cv::Point((endPosx/mapRezSim)*mapRezImg,(endPosy/mapRezSim)*mapRezImg),(endRadius/mapRezSim)*mapRezImg,yellow,1);

        scene.translateImg2(projectImgRGB,projectImgRGBMoved,offsetx,offsety);

        ofxCv::toOf(projectImgRGBMoved,projectImgOF);
        projectImgOF.update();
        scene.processScene();


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

    else if(calibration.isFinalized() && rendererInited)
    {

        renderer.drawImage(projectImgOF);
        //renderer.drawHueDepthImage();
        //Disable depth test after renderer call.
        ofDisableDepthTest();

        projectImgOF.draw(0,0,WIN_WIDTH,WIN_HEIGHT);

        //threshMaskOF[0].draw(WIN_WIDTH*0.7,0,WIN_WIDTH*0.3,WIN_HEIGHT*0.3);
        //threshMaskOF[1].draw(WIN_WIDTH*0.7,WIN_HEIGHT*0.3,WIN_WIDTH*0.3,WIN_HEIGHT*0.3);
        //threshMaskOF[2].draw(WIN_WIDTH*0.7,WIN_HEIGHT*0.6,WIN_WIDTH*0.3,WIN_HEIGHT*0.3);

        ofDrawBitmapString(ofToString(ofGetFrameRate())+" fps", 50, 190);

        if(!guiHide)
        {
            terrainControls.draw();
            simControls.draw();
            imageControls.draw();
        }

        if(showContours)
        {
            // Debug to draw contours
            obj.nativeDrawContours(0,0,WIN_WIDTH,WIN_HEIGHT);
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

    case 'h':
        guiHide=!guiHide;
        break;
    case 'a':
        cout<<"\nAdding new boids\n";
        sim.addAllBoids();
        break;

    case 'r':
        cout<<"\nRemove all boids\n";
        sim.removeAllBoids();

        break;


    }
}

void ofApp::mousePressed(int x, int y, int button)
{
    //cout<<"\nMOUSE: x y button "<<x<<" "<<y<<" "<<button;
    if(!calibration.isFinalized())
    {


        if(maskPoints>=4)
        {
            maskPoints=0;
            writeMask=true;
        }
        kinectMask[maskPoints]=cv::Point(x*2*KINECT_WIDTH/(WIN_WIDTH),y*2*KINECT_HEIGHT/(WIN_HEIGHT));
        drawkinectMask[maskPoints]=cv::Point(x,y);
        maskPoints++;
    }
    else if(calibration.isFinalized() && rendererInited)
    {
        if(button==0)
        {
            startPosx=x*IMG_WIDTH*mapRezSim/WIN_WIDTH;
            startPosy=y*IMG_HEIGHT*mapRezSim/WIN_HEIGHT;
            sim.setStart(startPosx,startPosy);
        }
        else if(button==2)
        {
            endPosx=x*IMG_WIDTH*mapRezSim/WIN_WIDTH;
            endPosy=y*IMG_HEIGHT*mapRezSim/WIN_HEIGHT;
            sim.setDestination(endPosx,endPosy);
        }
    }

}

void ofApp::exit()
{
}
