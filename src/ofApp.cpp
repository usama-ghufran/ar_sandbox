#include "ofApp.h"



void ofApp::setup(){

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

    maskPoints=0;

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


}

void ofApp::update(){
	depthcam.update();
	if(!calibration.isFinalized()) {
		calibration.update();
	}

	if(calibration.isFinalized() && !rendererInited) {
		renderer.init(&depthcam);
		renderer.setDrawArea(WIN_WIDTH,0,WIN_WIDTH,WIN_HEIGHT);
		renderer.setProjectionMatrix(dataset.getMatrix());

		rendererInited = true;
	}

	if(calibration.isFinalized() && rendererInited) {
		renderer.update();

	}
}

void ofApp::draw(){
	if(!calibration.isFinalized()) {
		calibration.drawStatusScreen(0,0,WIN_WIDTH,WIN_HEIGHT);
		calibration.drawChessboard(WIN_WIDTH,0,WIN_WIDTH,WIN_HEIGHT);

		//Draw Mask Points;
		ofSetColor(255,0,0);
        for(int i=0; i<maskPoints; i++)
        {
            ofCircle(kinectMask[i].x,kinectMask[i].y,3);
            ofDrawBitmapString(ofToString(i),kinectMask[i].x,kinectMask[i].y);
        }
        ofSetColor(255,255,255);


        //cv::fillConvexPoly())
	}

	if(calibration.isFinalized() && rendererInited) {
		//renderer.drawHueDepthImage();


        renderer.drawImage(depthcam.getPixels(),depthcam.width,depthcam.height);


	}
}

void ofApp::keyPressed(int key){
}

void ofApp::mousePressed(int x, int y, int button){

    if(!calibration.isFinalized()) {


    cout<<"\nMOUSE: x y button "<<x<<" "<<y<<" "<<button;

    if(maskPoints>=4)
        maskPoints=0;

    kinectMask[maskPoints]=ofPoint(x,y);
    maskPoints++;
    }

}



void ofApp::exit(){
}


