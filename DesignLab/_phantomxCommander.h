//#pragma once
//
//#include<iostream>
//#include <windows.h> 
//#include <stdio.h>
//#include <stdlib.h>
//#include <math.h>
//#include <sstream>
//#include <fstream>
//#include <string>
//
//#include "designlab_vector3.h"
//#include "TimeManager.h"
//
//
//#define PI 3.14159265358979
//
////setup id of leg
//#define RIGHT_FRONT		0
//#define RIGHT_MIDDLE	1
//#define RIGHT_REAR		2
//#define LEFT_REAR		3
//#define LEFT_MIDDLE		4
//#define LEFT_FRONT		5
//#define LEG_COUNT		6
//
////setup speed of each gait
//#define Moving_Speed_L			0x5f
//#define Moving_Speed_H			0x00
//
////setup id of servo
//#define LF_COXA 1
//#define RF_COXA 2
//#define LF_FEMUR 3
//#define RF_FEMUR 4
//#define LF_TIBIA 5
//#define RF_TIBIA 6
//#define LR_COXA 7
//#define RR_COXA 8
//#define LR_FEMUR 9
//#define RR_FEMUR 10
//#define LR_TIBIA 11
//#define RR_TIBIA 12
//#define LM_COXA 13
//#define RM_COXA 14
//#define LM_FEMUR 15
//#define RM_FEMUR 16
//#define LM_TIBIA 17
//#define RM_TIBIA 18
//
////setup the GetLength of hardware
////#define X_COXA      60	// MM between front and back legs /2		//4脚用?
//#define X_COXA      120	// MM between front and back legs /2			//6脚用?
//#define Y_COXA      60	// MM between front/back legs /2
//#define M_COXA      100	// MM between two middle legs /2
////#define L_COXA      52	// MM distance from coxa servo to femur servo
////#define L_FEMUR     66	// MM distance from femur servo to tibia servo
////#define L_TIBIA     130	// MM distance from tibia servo to foot
//
//#define STD_TRANSITION          32   //98 for ax-12 hexapod, 32 for ax-18f
//#define MOVING			((Xspeed > 5 || Xspeed < -5) || (Yspeed > 5 || Yspeed < -5) || (Rspeed > 0.05 || Rspeed < -0.05))
//#define SERVO_COUNT 18
//
////基準姿勢
//#define BODY_HEIGHT 150
//#define LIFT_HEIGHT 30
//#define FRL_ALIGNMENT_POSE_X 54
//#define FRL_ALIGNMENT_POSE_Y 148
//#define ML_ALIGNMENT_POSE_X 0
//#define ML_ALIGNMENT_POSE_Y 108
//
//#define read_serial 82		//'R'
//#define	write_serial 87		//'W'
//#define scan_serial 83		//'S'
//
////setup serial port
////#define DEFAULT_PORTNUM		3		//COM13
//#define DEFAULT_BAUDRATE	9600	//ビットレート。ビットレートを変更する際はポート側、及びArduino側のビットレートも合わせて変更すること。
//
//
//class phantomxCommander
//{
//public:
//	phantomxCommander();
//	double myRound(double r);							//四捨五入用の関数
//	void sendEndPoints(int movingTime);		//シリアル通信でPhantomXの目標脚先位置を送信する。
//	void SecurelySendEndPoints(int movingTime, int waitTime);
//	void loopSendEndPoints(int loopNum, int movingTime, int waitTime = 200);		//シリアル通信で同一の目標脚先位置を複数回送信する。
//	void demonstration1();
//	int move1Cycle2(double rads, double stride);
//
//	int presentTime;						//直前の脚先移動時間を記憶する変数。
//	int command[120];						//速度制御もするとき用
//	int commandnum;							//速度制御もするとき用
//	int lifted_leg;
//	designlab::Vector3 endpoints[LEG_COUNT];
//	TimeInfo commandTime;
//	HANDLE h;
//	unsigned long nn;
//	char pszBuf[1000];				//バッファ。シリアル通信で送られてきた値をここに入れる。
//	int sBuf[1];
//	int baudRate;
//	int cGuf;
//	DWORD dwErrors;
//	COMSTAT ComStat;
//	DWORD dwCount;
//	DWORD dwRead;
//	DCB dcb;
//	COMMTIMEOUTS cto;
//
//private:
//
//};
