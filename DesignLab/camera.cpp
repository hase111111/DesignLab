#define _USE_MATH_DEFINES
#include <iostream>
#include "camera.h"
#include "Define.h"

float cameraX=0,cameraY=70, cameraZ=0;  //カメラの座標
float HRotate = 0, VRotate = Define::MY_PI / 2.0f, TRotate = 0;	//カメラの視線の角度

VECTOR camera2_pos = VGet(0.0f, 100.0f, 0.0f);;						//カメラ２の位置
VECTOR Target      = VGet(0.0f,-1.0f,0.0f);;						//カメラ2の視点方向
VECTOR Target_c    = VAdd(Target, camera2_pos);;
VECTOR Up          = VGet(0.0f, 0.0f, 1.0f);;						//カメラ2のカメラ上方向




void Camera_position(char Buf[ 256 ])
{
	std::cout<<"視点移動モード";

		// 左右キーでカメラの水平方向回転値を変更
		if( Buf[ KEY_INPUT_A ] == 1 ){
			HRotate -= ROTATE_SPEED;
		}
		if( Buf[ KEY_INPUT_D ] == 1 ){
			HRotate += ROTATE_SPEED;
		}
			// 上下キーでカメラの垂直方向回転値を変更
		if( Buf[ KEY_INPUT_W ] == 1 ){
			VRotate += ROTATE_SPEED;
		}
		if( Buf[ KEY_INPUT_S ] == 1 ){
			VRotate -= ROTATE_SPEED;
		}

		// 2018/06/06 カメラの前後移動　追加
		if (Buf[KEY_INPUT_0] == 1) {
			cameraX -= MOVE_SPEED * 10;
		}
		if (Buf[KEY_INPUT_9] == 1) {
			cameraX += MOVE_SPEED * 10;
		}

		// 左右キーでカメラ位置値を変更
		if( Buf[ KEY_INPUT_LEFT ] == 1 ){
			cameraX -= MOVE_SPEED * cos(HRotate) ;
			cameraZ += MOVE_SPEED * sin(HRotate) ;
		}
		if( Buf[ KEY_INPUT_RIGHT ] == 1 ){
			cameraX += MOVE_SPEED * cos(HRotate) ;
			cameraZ -= MOVE_SPEED * sin(HRotate) ;
		}
		// 上下キーでカメラ位置値を変更
		if( Buf[ KEY_INPUT_UP ] == 1 ){
			cameraX += MOVE_SPEED * cos(VRotate) * sin(HRotate) ;
			cameraZ += MOVE_SPEED * cos(VRotate) * cos(HRotate) ;
			cameraY -= MOVE_SPEED * sin(VRotate);
		}
		if( Buf[ KEY_INPUT_DOWN ] == 1 ){
			cameraX -= MOVE_SPEED * cos(VRotate) * sin(HRotate) ;
			cameraZ -= MOVE_SPEED * cos(VRotate) * cos(HRotate) ;
			cameraY += MOVE_SPEED * sin(VRotate);
		}

		// カメラの位置と回転値をセット
		SetCameraPositionAndAngle( VGet( cameraX, cameraY, cameraZ ), VRotate, HRotate, TRotate ) ;
}

void Camera_position2(char Buf[ 256 ]){


	VECTOR Up_Target = VNorm(VCross(Up, Target));				//UpとTergetを外積し正規化したもの
	Up_Target = VScale(Up_Target, (float)CAMERA2_RSPEED);				//Up * Target
	VECTOR Up_Speed = VScale(Up, (float)CAMERA2_RSPEED);				//Up * Speed
	VECTOR Target_Speed = VScale(Target, (float)CAMERA2_RSPEED);		//Target * Speed
	VECTOR Up_Target_Speed = VScale(Up_Target, (float)CAMERA2_YSPEED);	//Up * Target * Speed

	// A Dキーでカメラのロール方向回転値を変更
	if (Buf[KEY_INPUT_A] == 1){
		//TRotate -= ROTATE_SPEED;
		Up = VNorm( VSub(Up , Up_Target));
	}	
	if (Buf[KEY_INPUT_D] == 1){
		//TRotate += ROTATE_SPEED;
		Up = VNorm( VAdd(Up, Up_Target));
	}
	// W Sキーでカメラのピッチ方向回転値を変更
	if (Buf[KEY_INPUT_W] == 1){
		//VRotate += ROTATE_SPEED * cos(TRotate);
		//HRotate += ROTATE_SPEED * sin(TRotate);
		Target = VSub(Target, Up_Speed);
		Up = VAdd(Up, Target_Speed);
		Target = VNorm(Target);
		Up = VNorm(Up);
	}
	if (Buf[KEY_INPUT_S] == 1){
		//VRotate -= ROTATE_SPEED * cos(TRotate);
		//HRotate -= ROTATE_SPEED * sin(TRotate);

		Target = VAdd(Target, Up_Speed);
		Up = VSub(Up, Target_Speed);
		Target = VNorm(Target);
		Up = VNorm(Up);
	}
	

	// 左右キーでカメラ位置値を変更
	if (Buf[KEY_INPUT_LEFT] == 1){
		if (Buf[KEY_INPUT_Z] == 1){
			camera2_pos = VSub(camera2_pos, VNorm(Up_Target));
		}else {
			Target = VNorm(VSub(Target, Up_Target_Speed));
		}
	}
	if (Buf[KEY_INPUT_RIGHT] == 1){
		if (Buf[KEY_INPUT_Z] == 1){
			camera2_pos = VAdd(camera2_pos, VNorm(Up_Target));
		}else {
			Target = VNorm(VAdd(Target, Up_Target_Speed));
		}
	}
	// 上下キーでカメラ位置値を変更
	if (Buf[KEY_INPUT_UP] == 1){
		if (Buf[KEY_INPUT_Z] == 1){
			camera2_pos = VAdd(camera2_pos, Up);
		}else {
			camera2_pos = VAdd(camera2_pos, Target);
		}
	}
	if (Buf[KEY_INPUT_DOWN] == 1){
		if (Buf[KEY_INPUT_Z] == 1){
			camera2_pos = VSub(camera2_pos, Up);
		}else {
			camera2_pos = VSub(camera2_pos, Target);
		}
	}
	Target_c = VAdd(Target, camera2_pos);
	// カメラの位置と回転値をセット
	//SetCameraPositionAndAngle(VGet(cameraX, cameraY, cameraZ), VRotate, HRotate, TRotate);
	SetCameraPositionAndTargetAndUpVec(camera2_pos, Target_c, Up);
}

void Camera_position3(VECTOR c_o_m){

	static bool kaisetu = 0;
	if(kaisetu == 0){
		std::cout<<"視点移動モード　Tracking\n";

		std::cout<<"前進　　W\n";
		std::cout<<"後退　　S\n";
		std::cout<<"右移動　D\n";
		std::cout<<"左移動　A\n";

		std::cout<<"ピッチ↑　R_SHIFT\n";
		std::cout<<"ピッチ↓　↓\n";
		std::cout<<"ヨー右　　→\n";
		std::cout<<"ヨー左　　←\n";
		std::cout << "一時的に操作できなくしてる(camera.cpp/camera_position3())2021/01/22hato";
		kaisetu = 1;
	}


			// 左右キーでカメラの水平方向回転値を変更
		//if( GetAsyncKeyState(VK_LEFT) & 0x8000 ){
		//	HRotate -= ROTATE_SPEED;
		//}
		//if( GetAsyncKeyState(VK_RIGHT) & 0x8000 ){
		//	HRotate += ROTATE_SPEED;
		//}
		//	// 上下キーでカメラの垂直方向回転値を変更
		//if( GetAsyncKeyState(VK_DOWN) & 0x8000 ){
		//	VRotate += ROTATE_SPEED;
		//}
		//if( GetAsyncKeyState(VK_RSHIFT) & 0x8000 ){
		//	VRotate -= ROTATE_SPEED;
		//}

		//// 2018/06/06 追加
		//if (GetAsyncKeyState('1') & 0x8000) {
		//	cameraX -= MOVE_SPEED;
		//}
		//if (GetAsyncKeyState('2') & 0x8000) {
		//	cameraX -= MOVE_SPEED;
		//}

		//// 左右キーでカメラ位置値を変更
		//if( GetAsyncKeyState('A') & 0x8000 ){
		//	cameraX -= MOVE_SPEED * cos(HRotate) ;
		//	cameraZ += MOVE_SPEED * sin(HRotate) ;
		//}
		//if( GetAsyncKeyState('D') & 0x8000 ){
		//	cameraX += MOVE_SPEED * cos(HRotate) ;
		//	cameraZ -= MOVE_SPEED * sin(HRotate) ;
		//}
		//// 上下キーでカメラ位置値を変更
		//if( GetAsyncKeyState('W') & 0x8000 ){
		//	cameraX += MOVE_SPEED * cos(VRotate) * sin(HRotate) ;
		//	cameraZ += MOVE_SPEED * cos(VRotate) * cos(HRotate) ;
		//	cameraY -= MOVE_SPEED * sin(VRotate);
		//}
		//if( GetAsyncKeyState('S') & 0x8000 ){
		//	cameraX -= MOVE_SPEED * cos(VRotate) * sin(HRotate) ;
		//	cameraZ -= MOVE_SPEED * cos(VRotate) * cos(HRotate) ;
		//	cameraY += MOVE_SPEED * sin(VRotate);
		//}


		// カメラの位置と回転値をセット
		//cameraZ = cameraZ+ (c_o_m.z - cameraZ)/10;
		//cameraX = cameraX+ (c_o_m.x - cameraX)/10;

		SetCameraPositionAndAngle( VGet( cameraX, cameraY, cameraZ ), VRotate, HRotate, TRotate ) ;
}

// 呼ばれてない
void Camera_position2(){
	return;
}