#ifndef _TEST_APP
#define _TEST_APP

#include "ofMain.h"
#include "AbstractRobotModel.h"

#define N_CAMERAS 2

class testApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void draw();
		
		void setupViewports();

		void keyPressed(int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);		
    
        ofLight	light;
		std::vector<ofEasyCam> cam;
		int viewpoint;

		std::vector<AbstractRobotModel*> model;
		AbstractRobotModel* target;

		//viewports
		ofRectangle viewMain;
		ofRectangle viewGrid[N_CAMERAS];
		int viewport;
		bool flag_fullscreen;
		bool flag_background;
};

enum VIEWPOINT_TYPE
{
	VIEWPOINT_MAIN = 0,
	VIEWPOINT_RIGHT = 1,
	VIEWPOINT_LEFT = 2
};

enum VIEWPORT_TYPE
{
	VIEW_MAIN = 0,
	VIEW_HMD = 1
};

#endif
