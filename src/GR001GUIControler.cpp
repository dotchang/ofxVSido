#include "GR001GUIControler.h"
#include "RobotModelUtil.h"
#include <map>

#include "ofxVSido.h"

void GR001GUIControler::setup(){
	model.setScaleNomalization(false);
	model.loadModel("GR001b.dae", false);
	searchRegistOrigin(model.getAssimpScene()->mRootNode, TransformationOrigin);
	searchPrintNode(model.getAssimpScene()->mRootNode);

	jointNameToJointIdMap.insert(std::pair<std::string, int>("WAIST", 0));
	jointNameToJointIdMap.insert(std::pair<std::string, int>("R_HIP_Y", 1));
	jointNameToJointIdMap.insert(std::pair<std::string, int>("R_HIP_R", 2));
	jointNameToJointIdMap.insert(std::pair<std::string, int>("R_HIP_P", 3));
	jointNameToJointIdMap.insert(std::pair<std::string, int>("R_KNEE_P", 4));
	jointNameToJointIdMap.insert(std::pair<std::string, int>("R_ANKLE_P", 5));
	jointNameToJointIdMap.insert(std::pair<std::string, int>("R_ANKLE_R", 6));
	jointNameToJointIdMap.insert(std::pair<std::string, int>("L_HIP_Y", 7));
	jointNameToJointIdMap.insert(std::pair<std::string, int>("L_HIP_R", 8));
	jointNameToJointIdMap.insert(std::pair<std::string, int>("L_HIP_P", 9));
	jointNameToJointIdMap.insert(std::pair<std::string, int>("L_KNEE_P", 10));
	jointNameToJointIdMap.insert(std::pair<std::string, int>("L_ANKLE_P", 11));
	jointNameToJointIdMap.insert(std::pair<std::string, int>("L_ANKLE_R", 12));
	jointNameToJointIdMap.insert(std::pair<std::string, int>("CHEST_P", 13));
	jointNameToJointIdMap.insert(std::pair<std::string, int>("NECK_Y", 14));
	jointNameToJointIdMap.insert(std::pair<std::string, int>("R_SHOULDER_P", 15));
	jointNameToJointIdMap.insert(std::pair<std::string, int>("R_SHOULDER_R", 16));
	jointNameToJointIdMap.insert(std::pair<std::string, int>("R_ELBOW_P", 17));
	jointNameToJointIdMap.insert(std::pair<std::string, int>("L_SHOULDER_P", 18));
	jointNameToJointIdMap.insert(std::pair<std::string, int>("L_SHOULDER_R", 19));
	jointNameToJointIdMap.insert(std::pair<std::string, int>("L_ELBOW_P", 20));

	//waist.set("waist", 0, -180, 180);
	rleg[0].set("R_HIP_Y", 0, -150, 30.023);
	rleg[1].set("R_HIP_R", 0, -90.0117, 90.0117);
	rleg[2].set("R_HIP_P", 0, -39.9925, 119.977);
	rleg[3].set("R_KNEE_P", 0, -130.004, 0);
	rleg[4].set("R_ANKLE_P", 0, -96.9964, 59.9887);
	rleg[5].set("R_ANKLE_R", 0, -44.9772, 90.0117);
	lleg[0].set("L_HIP_Y", 0, -30.023, 150);
	lleg[1].set("L_HIP_R", 0, -90.0117, 90.0117);
	lleg[2].set("L_HIP_P", 0, -119.977, 39.9925);
	lleg[3].set("L_KNEE_P", 0, -0, 130.004);
	lleg[4].set("L_ANKLE_P", 0, -59.9887, 94.9964);
	lleg[5].set("L_ANKLE_R", 0, -90.0117, 44.9772);
	chest.set("CHEST_P", 0, -94.9964, 0);
	neck.set("NECK_Y", 0, -49.9619, 49.9619);
	rarm[0].set("R_SHOULDER_P", 0, -150, 150);
	rarm[1].set("R_SHOULDER_R", 0, -150, 39.9925);
	rarm[2].set("R_ELBOW_P", 0, -130.004, 50.0192);
	larm[0].set("L_SHOULDER_P", 0, -150, 150);
	larm[1].set("L_SHOULDER_R", 0, -39.9925, 150);
	larm[2].set("L_ELBOW_P", 0, 50.0192, 130.004);

	gui.setup("GR001");
	//gui.add(waist);
	for(int i=0; i<6; i++){ gui.add(rleg[i]); rleg[i].addListener(this, &GR001GUIControler::update_joints); }
	for(int i=0; i<6; i++){ gui.add(lleg[i]); lleg[i].addListener(this, &GR001GUIControler::update_joints); }
	gui.add(chest); chest.addListener(this, &GR001GUIControler::update_joints);
	gui.add(neck); neck.addListener(this, &GR001GUIControler::update_joints);
	for(int i=0; i<3; i++){ gui.add(rarm[i]); rarm[i].addListener(this, &GR001GUIControler::update_joints); }
	for(int i=0; i<3; i++){ gui.add(larm[i]); larm[i].addListener(this, &GR001GUIControler::update_joints); }
	
	ttime.set("ttime", 100, 0, 100);
	gui.add(ttime);
	vsido_connect.set("connect", false);
	gui.add(vsido_connect);

	int baud = 115200; // 9600, 57600, 115200, 1000000
	vsido.setup("COM6", baud);
}

void GR001GUIControler::update_joints(float & v)
{
	//model.getAssimpScene()->mRootNode->FindNode("WAIST_LINK")->mTransformation = TransformationOrigin["WAIST_LINK"]*aiMatrix4x4().FromEulerAnglesXYZ(0,0,-waist*3.14/180.0);
	model.getAssimpScene()->mRootNode->FindNode("R_HIP_Y_LINK")->mTransformation = TransformationOrigin["R_HIP_Y_LINK"]*aiMatrix4x4().FromEulerAnglesXYZ(0,0,-rleg[0]*3.14/180.0);
	model.getAssimpScene()->mRootNode->FindNode("R_HIP_R_LINK")->mTransformation = TransformationOrigin["R_HIP_R_LINK"]*aiMatrix4x4().FromEulerAnglesXYZ( rleg[1]*3.14/180.0,0,0);
	model.getAssimpScene()->mRootNode->FindNode("R_HIP_P_LINK")->mTransformation = TransformationOrigin["R_HIP_P_LINK"]*aiMatrix4x4().FromEulerAnglesXYZ(0, rleg[2]*3.14/180.0,0);
	model.getAssimpScene()->mRootNode->FindNode("R_KNEE_P_LINK")->mTransformation = TransformationOrigin["R_KNEE_P_LINK"]*aiMatrix4x4().FromEulerAnglesXYZ(0, rleg[3]*3.14/180.0,0);
	model.getAssimpScene()->mRootNode->FindNode("R_ANKLE_P_LINK")->mTransformation = TransformationOrigin["R_ANKLE_P_LINK"]*aiMatrix4x4().FromEulerAnglesXYZ(0,-rleg[4]*3.14/180.0,0);
	model.getAssimpScene()->mRootNode->FindNode("R_ANKLE_R_LINK")->mTransformation = TransformationOrigin["R_ANKLE_R_LINK"]*aiMatrix4x4().FromEulerAnglesXYZ(-rleg[5]*3.14/180.0,0,0);
	model.getAssimpScene()->mRootNode->FindNode("L_HIP_Y_LINK")->mTransformation = TransformationOrigin["L_HIP_Y_LINK"]*aiMatrix4x4().FromEulerAnglesXYZ(0,0,-lleg[0]*3.14/180.0);
	model.getAssimpScene()->mRootNode->FindNode("L_HIP_R_LINK")->mTransformation = TransformationOrigin["L_HIP_R_LINK"]*aiMatrix4x4().FromEulerAnglesXYZ( lleg[1]*3.14/180.0,0,0);
	model.getAssimpScene()->mRootNode->FindNode("L_HIP_P_LINK")->mTransformation = TransformationOrigin["L_HIP_P_LINK"]*aiMatrix4x4().FromEulerAnglesXYZ(0,-lleg[2]*3.14/180.0,0);
	model.getAssimpScene()->mRootNode->FindNode("L_KNEE_P_LINK")->mTransformation = TransformationOrigin["L_KNEE_P_LINK"]*aiMatrix4x4().FromEulerAnglesXYZ(0,-lleg[3]*3.14/180.0,0);
	model.getAssimpScene()->mRootNode->FindNode("L_ANKLE_P_LINK")->mTransformation = TransformationOrigin["L_ANKLE_P_LINK"]*aiMatrix4x4().FromEulerAnglesXYZ(0, lleg[4]*3.14/180.0,0);
	model.getAssimpScene()->mRootNode->FindNode("L_ANKLE_R_LINK")->mTransformation = TransformationOrigin["L_ANKLE_R_LINK"]*aiMatrix4x4().FromEulerAnglesXYZ(-lleg[5]*3.14/180.0,0,0);
	model.getAssimpScene()->mRootNode->FindNode("CHEST_P_LINK")->mTransformation = TransformationOrigin["CHEST_P_LINK"]*aiMatrix4x4().FromEulerAnglesXYZ(0, chest*3.14/180.0,0);
	model.getAssimpScene()->mRootNode->FindNode("NECK_Y_LINK")->mTransformation = TransformationOrigin["NECK_Y_LINK"]*aiMatrix4x4().FromEulerAnglesXYZ(0,0, neck*3.14/180.0);
	model.getAssimpScene()->mRootNode->FindNode("R_SHOULDER_P_LINK")->mTransformation = TransformationOrigin["R_SHOULDER_P_LINK"]*aiMatrix4x4().FromEulerAnglesXYZ(0,-rarm[0]*3.14/180.0,0);
	model.getAssimpScene()->mRootNode->FindNode("R_SHOULDER_R_LINK")->mTransformation = TransformationOrigin["R_SHOULDER_R_LINK"]*aiMatrix4x4().FromEulerAnglesXYZ(-rarm[1]*3.14/180.0,0,0);
	model.getAssimpScene()->mRootNode->FindNode("R_ELBOW_P_LINK")->mTransformation = TransformationOrigin["R_ELBOW_P_LINK"]*aiMatrix4x4().FromEulerAnglesXYZ(0,-rarm[2]*3.14/180.0,0);
	model.getAssimpScene()->mRootNode->FindNode("L_SHOULDER_P_LINK")->mTransformation = TransformationOrigin["L_SHOULDER_P_LINK"]*aiMatrix4x4().FromEulerAnglesXYZ(0, larm[0]*3.14/180.0,0);
	model.getAssimpScene()->mRootNode->FindNode("L_SHOULDER_R_LINK")->mTransformation = TransformationOrigin["L_SHOULDER_R_LINK"]*aiMatrix4x4().FromEulerAnglesXYZ(-larm[1]*3.14/180.0,0,0);
	model.getAssimpScene()->mRootNode->FindNode("L_ELBOW_P_LINK")->mTransformation = TransformationOrigin["L_ELBOW_P_LINK"]*aiMatrix4x4().FromEulerAnglesXYZ(0, larm[2]*3.14/180.0,0);
	model.update();

	if(vsido_connect){
		vector<int> id;
		vector<double> q;
		for(int i=0; i<6; i++){ if(last_rleg[i] != rleg[i]){ id.push_back(i); q.push_back(rleg[i]); } }
		for(int i=0; i<6; i++){ if(last_lleg[i] != lleg[i]){ id.push_back(i+6); q.push_back(lleg[i]); } }
		if(last_chest != chest){ id.push_back(12); q.push_back(chest); }
		if(last_neck != neck){ id.push_back(13); q.push_back(neck); }
		for(int i=0; i<3; i++){ if(last_rarm[i] != rarm[i]){ id.push_back(i+14); q.push_back(rarm[i]); } }
		for(int i=0; i<3; i++){ if(last_larm[i] != larm[i]){ id.push_back(i+17); q.push_back(larm[i]); } }
		vsido.setJointAngleCommand(id, q, ttime);
	}

	for(int i=0; i<6; i++){ last_rleg[i] = rleg[i]; }
	for(int i=0; i<6; i++){ last_lleg[i] = lleg[i]; }
	last_chest = chest;
	last_neck = neck;
	for(int i=0; i<3; i++){ last_rarm[i] = rarm[i]; }
	for(int i=0; i<3; i++){ last_larm[i] = larm[i]; }
}

void GR001GUIControler::update(){
}

void GR001GUIControler::draw(){
	//model.drawFaces();

	for(int i=0; i<model.getMeshCount(); i++){
		ofPushMatrix();
		ofMultMatrix(model.getModelMatrix());
		ofxAssimpMeshHelper & meshHelper = model.getMeshHelper(i);
		ofMultMatrix(meshHelper.matrix);
		ofSetColor( meshHelper.material.getDiffuseColor() );
		model.getMesh(i).draw();
		ofPopMatrix();
	}
	ofDisableDepthTest();
	ofDisableLighting();
	for(int i=0; i<model.getMeshCount(); i++){
		ofPushMatrix();
		ofMultMatrix(model.getModelMatrix());
		ofxAssimpMeshHelper & meshHelper = model.getMeshHelper(i);
		ofMultMatrix(meshHelper.matrix);
		ofDrawAxis(0.01);
		ofPopMatrix();
	}
	ofEnableLighting();
	ofEnableDepthTest();
	ofScale(1.0,1.0,1.0);
}

void GR001GUIControler::draw_gui(){
	ofDisableDepthTest();
	ofDisableLighting();
	gui.draw();
	ofEnableLighting();
	ofEnableDepthTest();
}

void GR001GUIControler::mouseDragged(int x, int y, int button, ofEasyCam& cam)
{
	if(button == 1 && !nearestIndex.empty()){
		cam.disableMouseInput();	
		float value_x = x / (float)ofGetWidth();
		float value_y = y / (float)ofGetHeight();

		for(int i=0; i<(int)nearestIndex.size(); i++){
			//aiString name = model.getMeshHelper(nearestIndex[i]).mesh->mName;
			//if(name == aiString("WAIST_LINK")){ waist = adjust_value(waist, value_x); }
			if(nearestIndex[i] == jointNameToJointIdMap["R_HIP_Y"]){ rleg[0] = adjust_value(rleg[0], value_x); }
			else if(nearestIndex[i] == jointNameToJointIdMap["R_HIP_R"]){ rleg[1] = adjust_value(rleg[1], value_x); }
			else if(nearestIndex[i] == jointNameToJointIdMap["R_HIP_P"]){ rleg[2] = adjust_value(rleg[2], value_y); }
			else if(nearestIndex[i] == jointNameToJointIdMap["R_KNEE_P"]){ rleg[3] = adjust_value(rleg[3], value_y); }
			else if(nearestIndex[i] == jointNameToJointIdMap["R_ANKLE_P"]){ rleg[4] = adjust_value(rleg[4], value_y); }
			else if(nearestIndex[i] == jointNameToJointIdMap["R_ANKLE_R"]){ rleg[5] = adjust_value(rleg[5], value_x); }
			else if(nearestIndex[i] == jointNameToJointIdMap["L_HIP_Y"]){ lleg[0] = adjust_value(lleg[0], value_x); }
			else if(nearestIndex[i] == jointNameToJointIdMap["L_HIP_R"]){ lleg[1] = adjust_value(lleg[1], value_x); }
			else if(nearestIndex[i] == jointNameToJointIdMap["L_HIP_P"]){ lleg[2] = adjust_value(lleg[2], value_y); }
			else if(nearestIndex[i] == jointNameToJointIdMap["L_KNEE_P"]){ lleg[3] = adjust_value(lleg[3], value_y); }
			else if(nearestIndex[i] == jointNameToJointIdMap["L_ANKLE_P"]){ lleg[4] = adjust_value(lleg[4], value_y); }
			else if(nearestIndex[i] == jointNameToJointIdMap["L_ANKLE_R"]){ lleg[5] = adjust_value(lleg[5], value_x); }
			else if(nearestIndex[i] == jointNameToJointIdMap["CHEST_P"]){ chest = adjust_value(chest, value_y); }
			else if(nearestIndex[i] == jointNameToJointIdMap["NECK_Y"]){ neck = adjust_value(neck, value_x); }
			else if(nearestIndex[i] == jointNameToJointIdMap["R_SHOULDER_P"]){ rarm[0] = adjust_value(rarm[0], value_x); }
			else if(nearestIndex[i] == jointNameToJointIdMap["R_SHOULDER_R"]){ rarm[1] = adjust_value(rarm[1], value_x); }
			else if(nearestIndex[i] == jointNameToJointIdMap["R_ELBOW_P"]){ rarm[2] = adjust_value(rarm[2], value_x); }
			else if(nearestIndex[i] == jointNameToJointIdMap["L_SHOULDER_P"]){ larm[0] = adjust_value(larm[0], value_x); }
			else if(nearestIndex[i] == jointNameToJointIdMap["L_SHOULDER_R"]){ larm[1] = adjust_value(larm[1], value_x); }
			else if(nearestIndex[i] == jointNameToJointIdMap["L_ELBOW_P"]){ larm[2] = adjust_value(larm[2], value_x); }
		}
	}
}

int GR001GUIControler::getCameraPosition(int id, ofEasyCam& cam)
{
	switch(id){
	case 0: // Normal
	case 1:
	case 2:
		cam.setTarget(model.getPosition());
		cam.lookAt(model.getPosition(),ofVec3f(0,-1,0));
		break;
	default:
		return -1;
	}
	return 0;
}

void GR001GUIControler::mousePressed(int x, int y, int button, ofEasyCam& cam)
{
	if(button==1){
		int n = model.getMeshCount();
		float nearestDistance = 0;
		ofVec2f nearestVertex;
		nearestIndex.clear();
		ofVec2f mouse(x,y);
		for(int i=0; i<n; i++){
			ofMatrix4x4 m = model.getMeshHelper(i).matrix * model.getModelMatrix();
			ofVec3f cur = cam.worldToScreen(m.getTranslation());
			float distance = cur.distance(mouse);
			if(i == 0|| distance < nearestDistance){
				nearestDistance = distance;
				nearestVertex = cur;
				nearestIndex.clear();
				nearestIndex.push_back(i);
			}
			else if( fabs(distance-nearestDistance) < 10){
				nearestIndex.push_back(i);
			}
		}
		if(nearestDistance > 20){
			nearestIndex.clear();
		}
		for(int i=0; i<(int)nearestIndex.size(); i++){
			cout << "Nearest Index = " << nearestIndex[i] << ", nearestDistance = " << nearestDistance << endl;
		}
	}
}