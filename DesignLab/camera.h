#pragma once
#include "pch.h"

#define ROTATE_SPEED		(DX_PI_F/60) //カメラ1の回転スピード
#define CAMERA2_RSPEED		0.05	//カメラ2のロールスピード
#define CAMERA2_YSPEED		0.2		//カメラ2のヨー  スピード
#define MOVE_SPEED			1.0f			//カメラの移動速度


//カメラ位置関数
void Camera_position(char Buf[ 256 ]);
//カメラ位置関数　戦闘機ver
void Camera_position2(char Buf[ 256 ]);
//カメラ位置関数　追尾ver
void Camera_position3(VECTOR c_o_m);

//カメラ位置関数　戦闘機ver
void Camera_position2();

