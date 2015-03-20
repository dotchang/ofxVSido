#include "ofxVSido.h"

#define _USE_MATH_DEFINES
#include <math.h>

ofxVSido::ofxVSido()
	: semaphore(0)
{
}

int ofxVSido::setup(int deviceNumber, int baudrate)
{
	serial.setup(deviceNumber, baudrate);
	Sleep(100);
	serial.flush();
	return serial.available();
}

int ofxVSido::setup(std::string portName, int baudrate)
{
	serial.setup(portName, baudrate);
	Sleep(100);
	serial.flush();
	return serial.available();
}

int ofxVSido::setup(ofSerial& _serial)
{
	serial = _serial;
	Sleep(100);
	serial.flush();
	return serial.available();
}

inline double degree(double rad) { return (180.0 * rad / M_PI); }

// from Choreonoid GRobotController.cpp
const char jointIdToMotorMap[] = {
	8 /* R_HIP_Y */, 10 /* R_HIP_R */, 9 /* R_HIP_P */, 11 /* R_KNEE_P */, 12 /* R_ANKLE_P */, 13 /* R_ANKLE_R */,
	14 /* L_HIP_Y */, 16 /* L_HIP_R */, 15 /* L_HIP_P */, 17 /* L_KNEE_P */, 18 /* L_ANKLE_P */, 19 /* L_ANKLE_R */,
	0 /* CHEST_P */, 1 /* NECK_Y */,
	2 /* R_SHOULDER_P */, 3 /* R_SHOULDER_R */, 4 /* R_ELBOW_P */,
	5 /* R_SHOULDER_P */, 6 /* R_SHOULDER_R */, 7 /* R_ELBOW_P */ };

char calcSum(unsigned char* data, int len)
{
	char sum = 0;
	for(int i=0; i < len; ++i){
		sum ^= data[i];
	}
	return sum;
}

void ofxVSido::readBuffer(vector<unsigned char>& buffer)
{
	for(int i=0;;i++){
		int c = serial.readByte();
		
		if(c>0){
			buffer.push_back((unsigned char)c);
			unsigned char sum = calcSum(buffer.data(), buffer.size()-1);
			//cout << i << " " << c << " " << buffer.size() << " " << (int)sum << endl;
		}
		else if(c<0) break; // error
		
		if(buffer.size()>=2){
			if(buffer[2] == buffer.size()) break;
		}
		else if(i>1000000){
			break;
		}
	}
}

int ofxVSido::setJointAngleCommand(vector<int> jointId, vector<double> q, double ttime)
{
	if(!serial.isInitialized()) return -1;
	serial.flush();
	if(semaphore) return -2;
	semaphore = 1;

	vector<unsigned char> command;
	int num = jointId.size();
	command.push_back(0xff);													// ST
	command.push_back(0x6f);													// 'o'
	command.push_back(5+3*num);													// LN
	command.push_back((short)ttime);											// CYC
	for(int i=0; i<num; i++){
		int motorId = jointIdToMotorMap[jointId[i]] + 1;
		command.push_back(motorId);												// SID	
		short qcom = (short)(q[i] * 10.0) << 1;
		command.push_back(qcom & 0xff);											// ANGLE
		command.push_back(((qcom>>8)<<1) & 0xff);								// ANGLE
	}
	command.push_back(calcSum((unsigned char*)command.data(), command.size())); // SUM

	serial.writeBytes(command.data(), command.size());

	vector<unsigned char> buffer;
	readBuffer( buffer );

#if _DEBUG
	if(buffer.size() > 2){
		unsigned char c = calcSum((unsigned char*)buffer.data(), buffer[2]-1);
		cout << 0xff << " " << 0x21 << " " << (int)buffer[2] << " " << (int)c << endl;
	}
#endif

	semaphore = 0;
	return 0;
}
