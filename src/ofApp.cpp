#include "ofApp.h"
#include "ImgProc.h"
#include "ofxCvGrayscaleImage.h"
using namespace ofxCv;
using namespace cv;
ImgProc obj;

void ofApp::setup(){

        obj.img.loadImage("obstacles.png");
        obj.cvimg.setFromPixels(obj.img.getPixels(),obj.img.getWidth(),obj.img.getHeight());

        arguments = ofxArgParser::allKeys();
        ofSetLogLevel(OF_LOG_WARNING);
        ofSetVerticalSync(false);

        depthcam.setRegistration(true);
        depthcam.init();
        depthcam.setUseTexture(false);
        depthcam.open();
        dataset.loadFile("calibrationData.xml");

        calibration.init(&depthcam,&dataset);
        calibration.enableKeys();
        calibration.enableChessboardMouseControl();

        rendererInited = false;
        writeMask=true;
        mapRez=1;
        maskPoints=0;

        mask=cv::imread("data/mask.bmp",1);
        flockImg=cv::Mat(1080,1920,CV_8UC3,cv::Scalar(0,0,0));
        flockingImgOF.allocate(1920,1080,OF_IMAGE_COLOR);

        if(!mask.data)
                mask = cv::Mat(1080,1920,CV_8UC3,cv::Scalar(0,0,0));

        //Parse Args
        for (int i = 0; i < arguments.size(); i++){
                if(arguments[i]=="calib"){
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
sim.loadScene(30,30,600,300,1920/mapRez,1080/mapRez);
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
        50
        );

        flockDisplay = sim.getFlockHandle();
}

void ofApp::update(){

depthcam.update();
cv::Scalar white(255,255,255);
cv::Scalar black(0,0,0);

if(!calibration.isFinalized()) {
        calibration.update();
        if(maskPoints==4 && writeMask){
                    mask = black;
                    cv::fillConvexPoly(mask,kinectMask,4,white);
                    cv::imwrite("data/mask.bmp",mask);
                    writeMask=false;
        }
}

if(calibration.isFinalized() && !rendererInited) {
        renderer.init(&depthcam);
	renderer.setDrawArea(WIN_WIDTH,0,WIN_WIDTH,WIN_HEIGHT);
	renderer.setProjectionMatrix(dataset.getMatrix());
	rendererInited = true;
}

if(calibration.isFinalized() && rendererInited) {
        //renderer.update();
        sim.frame();
        obj.processImage();
        vector<Boid>* boids = flockDisplay->getBoidsHandle();
        flockImg=black;

        for(int i=0;i<boids->size();i++){
            float x = (*boids)[i].loc.x*mapRez;
            float y = (*boids)[i].loc.y*mapRez;
            cv::circle(flockImg,cv::Point(x,y),3,white,-1);
            obj.boidCollision( (*boids)[i]);
        }
        ofxCv::toOf(flockImg,flockingImgOF);
        flockingImgOF.update();
}
}

void ofApp::draw(){
if(!calibration.isFinalized()) {
        calibration.drawStatusScreen(0,0,WIN_WIDTH,WIN_HEIGHT);
        calibration.drawChessboard(WIN_WIDTH,0,WIN_WIDTH,WIN_HEIGHT);
        //Draw Mask Points;
        ofSetColor(255,0,0);

        for(int i=0; i<maskPoints; i++) {
            ofCircle(drawkinectMask[i].x,drawkinectMask[i].y,3);
            ofDrawBitmapString(ofToString(i),drawkinectMask[i].x,drawkinectMask[i].y);
        }
        ofSetColor(255,255,255);
}

if(calibration.isFinalized() && rendererInited) {
        //renderer.drawHueDepthImage();
/*
vector<Boid>* boids = flockDisplay->getBoidsHandle();
ofSetColor(0,0,255);
for(int i=0;i<boids->size();i++)
{
    ofCircle((*boids)[i].loc.x*mapRez,(*boids)[i].loc.y*mapRez,2);
}
ofSetColor(0,0,255);
*/
renderer.drawImage(flockingImgOF);
flockingImgOF.draw(0,0);

}



}

void ofApp::keyPressed(int key){

switch (key){
        case OF_KEY_ESC:
            maskPoints=0;
        break;
}
}

void ofApp::mousePressed(int x, int y, int button){

if(!calibration.isFinalized()) {
    //cout<<"\nMOUSE: x y button "<<x<<" "<<y<<" "<<button;

        if(maskPoints>=4){
                maskPoints=0;
                writeMask=true;
        }
        kinectMask[maskPoints]=cv::Point(x*2*1920/(WIN_WIDTH),y*2*1080/(WIN_HEIGHT));
        drawkinectMask[maskPoints]=cv::Point(x,y);
        maskPoints++;
}
}

void ofApp::exit(){
}
