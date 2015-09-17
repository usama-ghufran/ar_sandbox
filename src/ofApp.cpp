#include "ofApp.h"

bool rendererInited = false;

void ofApp::setup(){
	ofSetLogLevel(OF_LOG_VERBOSE);
	ofSetVerticalSync(false);

    depthcam.setRegistration(true);
	depthcam.init();
	depthcam.setUseTexture(false);
	depthcam.open();

	dataset.loadFile("exampleCalibrationData.xml");

	calibration.init(&depthcam,&dataset);
	calibration.enableKeys();
	calibration.enableChessboardMouseControl();

	rendererInited = false;

}

void ofApp::update(){
	depthcam.update();
	if(!calibration.isFinalized()) {
		calibration.update();
	}

	if(calibration.isFinalized() && !rendererInited) {
		renderer.init(&depthcam);
		renderer.setDrawArea(1920,0,1920,1080);
		renderer.setProjectionMatrix(dataset.getMatrix());

		rendererInited = true;
	}

	if(calibration.isFinalized() && rendererInited) {
		renderer.update();
	}
}

void ofApp::draw(){
	if(!calibration.isFinalized()) {
		calibration.drawStatusScreen(0,0,1920,1080);
		calibration.drawChessboard(1920,0,1920,1080);
	}

	if(calibration.isFinalized() && rendererInited) {
		renderer.drawHueDepthImage();
	}
}

void ofApp::keyPressed(int key){
}

void ofApp::exit(){
}


