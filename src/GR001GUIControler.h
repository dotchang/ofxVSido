#ifndef _GR001_GUI_CONTROLER_H
#define _GR001_GUI_CONTROLER_H

#include "AbstractRobotModel.h"
#include "ofxAssimpModelLoader.h"
#include "ofxGui.h"
#include <map>
#include "ofxVSido.h"

class GR001GUIControler : AbstractRobotModel
{
public:
	void setup();
	void update();
	void draw();
	void draw_gui();

	void keyPressed(int key){};
	void mouseDragged(int x, int y, int button, ofEasyCam& cam);
	void mousePressed(int x, int y, int button, ofEasyCam& cam);

	int getCameraPosition(int id, ofEasyCam& cam);
	ofxAssimpModelLoader* getModel(){ return &model; }
	void update_joints(float & v);

	ofxAssimpModelLoader model;
	std::map<std::string, aiMatrix4x4> TransformationOrigin;
	std::vector<int> nearestIndex;

	ofxPanel gui;
	ofParameter<float> waist, chest, neck, rleg[6], lleg[6], rarm[3], larm[3];
	ofParameter<float> last_waist, last_chest, last_neck, last_rleg[6], last_lleg[6], last_rarm[3], last_larm[3];
	ofParameter<int> ttime;

	std::map<std::string, int> jointNameToJointIdMap;
	ofParameter<bool> vsido_connect;

	ofxVSido vsido;
};

#endif