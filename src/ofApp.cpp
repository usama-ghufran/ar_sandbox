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
	writeMask=true;

    maskPoints=0;
    mask=cv::imread("data/mask.bmp",1);

    if(!mask.data)
        mask = cv::Mat(1080,1920,CV_8UC3,cv::Scalar(0,0,0));

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

		if(maskPoints==4&&writeMask)
        {
            cv::Scalar white(255,255,255);
            mask = cv::Scalar(0,0,0);
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
            ofCircle(drawkinectMask[i].x,drawkinectMask[i].y,3);
            ofDrawBitmapString(ofToString(i),drawkinectMask[i].x,drawkinectMask[i].y);
        }
        ofSetColor(255,255,255);



	}

	if(calibration.isFinalized() && rendererInited) {
		//renderer.drawHueDepthImage();


         renderer.drawImage(depthcam.getPixels(),depthcam.width,depthcam.height);


	}
}

void ofApp::keyPressed(int key){

    switch (key)
    {
        case OF_KEY_ESC:
            maskPoints=0;
        break;


    }


}

void ofApp::mousePressed(int x, int y, int button){

    if(!calibration.isFinalized()) {


    //cout<<"\nMOUSE: x y button "<<x<<" "<<y<<" "<<button;

    if(maskPoints>=4)
    {
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


