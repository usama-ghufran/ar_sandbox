#include "ofApp.h"
#include "ofAppGLFWWindow.h"

int main() {
	ofAppGLFWWindow window;
	ofSetupOpenGL(&window, 1920*2, 1080, OF_FULLSCREEN);
	ofRunApp(new ofApp());

}
