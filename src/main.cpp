#include "ofApp.h"
#include "ofAppGLFWWindow.h"

int main() {
	ofAppGLFWWindow window;
	ofSetupOpenGL(&window, WIN_WIDTH*2, WIN_HEIGHT, OF_FULLSCREEN);
	ofRunApp(new ofApp());

}
