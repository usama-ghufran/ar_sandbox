#include "ofApp.h"
#include "ofAppGLFWWindow.h"
#include "ofxArgParser.h"

int main(int argc,const char** argv ) {

	ofxArgParser::init(argc, argv);
	ofAppGLFWWindow window;
	ofSetupOpenGL(&window, WIN_WIDTH, WIN_HEIGHT,OF_WINDOW);// OF_FULLSCREEN);
	ofRunApp(new ofApp());

}
