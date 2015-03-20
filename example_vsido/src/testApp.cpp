#include "testApp.h"
#include "aiScene.h"

#define _USE_GR001
#ifdef _USE_GR001
#include "GR001GUIControler.h"
GR001GUIControler gr001;
#endif

//--------------------------------------------------------------
void testApp::setup(){
	ofSetLogLevel(OF_LOG_VERBOSE);
	ofBackground(50, 0);
	ofSetWindowShape(1280,960);

	ofMatrix4x4 origin;
	origin.makeIdentityMatrix();

	ofMatrix4x4 pos;

#ifdef _USE_GR001
	gr001.setup();
	target = (AbstractRobotModel*)&gr001;
	pos.setTranslation(-0.5,-0.5,0);
	pos = pos*origin; 
	gr001.model.setPosition(pos.getTranslation().x,pos.getTranslation().y,pos.getTranslation().z);
	gr001.gui.setPosition(800,10);
	model.push_back((AbstractRobotModel*)&gr001);
#endif

	ofEnableBlendMode(OF_BLENDMODE_ALPHA);

	ofEnableDepthTest();

	glShadeModel(GL_SMOOTH); //some model / light stuff
	light.enable();
	ofEnableSeparateSpecularLight();

	cam.resize(3);
	for(int i=0; i<cam.size(); i++){
		cam[i].setFov(80);
		cam[i].setScale(0.001);
		cam[i].setPosition((float)ofGetWidth() * -0.5, (float)ofGetHeight() * -0.5 , 0);
		cam[i].setTarget(target->getModel()->getPosition());
		cam[i].lookAt(target->getModel()->getPosition(),ofVec3f(0,-1,0));
		cam[i].setDistance(0.5);
	}

	viewpoint = 0;
	viewport = VIEW_MAIN; //VIEW_HMD;
}

//--------------------------------------------------------------
void testApp::setupViewports(){
	//call here whenever we resize the window


	//--
	// Define viewports

	float xOffset = ofGetWidth() / N_CAMERAS;
	float yOffset = ofGetHeight(); // / N_CAMERAS;

	viewMain.x = 0; //xOffset;
	viewMain.y = 0;
	viewMain.width = ofGetWidth(); //xOffset * 2;
	viewMain.height = ofGetHeight();

	for(int i = 0; i < N_CAMERAS; i++){

		viewGrid[i].x = xOffset * i;
		viewGrid[i].y = 0; //yOffset * i;
		viewGrid[i].width = xOffset;
		viewGrid[i].height = yOffset;
	}

	//
	//--
}

//--------------------------------------------------------------
void testApp::update(){
	for(int i=0; i<model.size(); i++){
		ofScale(1.0,1.0,1.0);
		model[i]->update();
	}
	if(viewport == VIEW_MAIN){
		target->getCameraPosition(viewpoint,cam[viewpoint]);
	}
	else if(viewport == VIEW_HMD){
		target->getCameraPosition(VIEWPOINT_LEFT,cam[VIEWPOINT_LEFT]);
		target->getCameraPosition(VIEWPOINT_RIGHT,cam[VIEWPOINT_RIGHT]);
	}
}

//--------------------------------------------------------------
void testApp::draw(){
	ofSetColor(255);

	if(viewport == VIEW_MAIN){
		ofBackground(50, 0);
		cam[viewpoint].begin(viewMain);
		for(int i=0; i<model.size(); i++){
			model[i]->draw();
		}
		cam[viewpoint].end();

		for(int i=0; i<model.size(); i++){
			model[i]->draw_gui();
		}

		//
		ofPushStyle();
		glDepthFunc(GL_ALWAYS); // draw on top of everything
		ofNoFill();
		ofSetColor(255, 255, 255);
		ofRect(viewMain);
		glDepthFunc(GL_LESS);
		ofPopStyle();
	}
	else if(viewport == VIEW_HMD){
		ofBackground(0, 0);
		cam[VIEWPOINT_RIGHT].begin(viewGrid[0]);
		for(int i=0; i<model.size(); i++){
			model[i]->draw();
		}
		cam[VIEWPOINT_RIGHT].end();

		cam[VIEWPOINT_LEFT].begin(viewGrid[1]);
		for(int i=0; i<model.size(); i++){
			model[i]->draw();
		}
		cam[VIEWPOINT_LEFT].end();

		//
		ofPushStyle();
		glDepthFunc(GL_ALWAYS); // draw on top of everything
		ofNoFill();
		for(int i=0; i<N_CAMERAS; i++){
			ofSetColor(255, 255, 255);
			ofRect(viewGrid[i]);
		}
		glDepthFunc(GL_LESS);
		ofPopStyle();
	}
}

//--------------------------------------------------------------
void testApp::keyPressed(int key){
	switch(key){

	case 259: // F3
#ifdef _USE_GR001
		target = (AbstractRobotModel*)&gr001;
#endif
		break;
	case 261: // F5
		viewport = VIEW_MAIN;
		break;
	case 262: // F6
		viewport = VIEW_HMD;
		break;
	case 'f': // F
		flag_fullscreen = !flag_fullscreen;
		ofSetFullscreen(flag_fullscreen);
		break;
	case 'b': // b
		flag_background = !flag_background;
		break;
	case '0': viewpoint = 0; break; // Normal View
	case '1': viewpoint = 1; break; // Left Camera View
	case '2': viewpoint = 2; break; // Right Camera View
	default: break;
	}
	target->keyPressed(key);
}

//--------------------------------------------------------------
void testApp::keyReleased(int key){
	//
}

//--------------------------------------------------------------
void testApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void testApp::mouseDragged(int x, int y, int button){
	for(int i=0; i<model.size(); i++){
		model[i]->mouseDragged(x,y,button,cam[viewpoint]);
	}
}

//--------------------------------------------------------------
void testApp::mousePressed(int x, int y, int button){
	for(int i=0; i<model.size(); i++){
		model[i]->mousePressed(x,y,button,cam[viewpoint]);
	}
}

//--------------------------------------------------------------
void testApp::mouseReleased(int x, int y, int button){
	cam[viewpoint].enableMouseInput();
}

//--------------------------------------------------------------
void testApp::windowResized(int w, int h){
	setupViewports();
}

//--------------------------------------------------------------
void testApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void testApp::dragEvent(ofDragInfo dragInfo){

}
