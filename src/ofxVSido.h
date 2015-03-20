#pragma once

#include "ofSerial.h"
#include "ofParameter.h"
#include <vector>

class ofxVSido
{
public:
	ofxVSido();
	
	int setup(int deviceNumber, int baudrate);
	int setup(std::string , int baudrate);
	int setup(ofSerial& _serial);

	int setJointAngleCommand(std::vector<int> jointId, std::vector<double> q, double ttime);
	void readBuffer(vector<unsigned char>& buffer);

	ofSerial serial;
	int semaphore;
};
