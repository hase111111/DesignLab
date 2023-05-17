#pragma once

#include "listFunc.h"
#include "hexapod.h"
#include "camera.h"
#include "Define.h"
#include "HexapodConst.h"
#include "MapConst.h"
//#include "mapData.h"
#include <vector>
#include <boost/thread.hpp>

#define THICKNESS			19.0f		//脚接地可能点の厚さ
#define LEG_R				7.5f		//脚の半径
#define TIBIA_LENGTH		140.0f		//TIBIAの長さ
#define FOOTHOLD_R			10.0f		//脚接地可能点の半径
#define FOOTHOLD_R_GROUND	10.0f		//脚が設置してる点の半径
#define MAGNIFICATION		0.025f		//拡大率  計算は㎜単位　描写はちょうどいい感じに拡大


class HexapodGraphic
{
public:
	HexapodGraphic(HINSTANCE hInstance);
	~HexapodGraphic() = default;
	void operator()();					//これがメインとなる関数

	//スレッド間の通信用
	std::vector<LNODE> *answer;			//結果を格納するベクタ
	int mapData_Max;
	boost::mutex *mtxHexapodGraphic, *mtxMapData;		//ミューテックス Data通信用
	myvector::SVector *p_map;
	bool *isCicle;						//0でメインの関数を抜ける
	
	//コールバック関数
	static LRESULT CALLBACK WndProcGraphic(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam);

private:
	bool is_grounded[MapConst::MAPDATA3D_MAX];//着地した脚設置可能点を記録する
	LNODE newPose, previousPose;
	void DrawHexapod(VECTOR c_o_m, myvector::SVector leg_position[6], float pitch, float roll, float yaw);	//6足ロボットの描写
	void DrawHexagon(VECTOR c_o_m, float Fwidth, float Mwidth, float Rwidth, float Flength, float Rlength, float thickness, COLOR_U8 color);//6角形の描写
	void DrawBody(VECTOR v0, VECTOR v1, VECTOR v2, VECTOR v3, VECTOR v4, VECTOR v5, COLOR_U8 color);
	void DrawGround();
	void DrawPossibleLegPosi(myvector::SVector leg_position[6]);
	void DrawMessage();
	void DrawSupportPolygon(myvector::SVector leg_position[6], VECTOR c_o_m, float pitch, float roll, float yaw);	//支持脚多角形の描写
	void DrawRangeOfDirection(VECTOR c_o_m, float pitch, float roll, float yaw);	//カメラによる検出範囲
	void DrawComPass(VECTOR c_o_m);	//重心移動軌跡
	Hexapod phantomX;
	//グラフィック上での６足ロボットの設定
	const COLOR_U8 CrBody;	//六角形ボディの色
	COLOR_U8 CrGround, CrGround2, CrJoint[6];
	unsigned int m_color_leg[HexapodConst::LEG_NUM];

	VECTOR map[MapConst::MAPDATA3D_MAX];						//脚接地可能点格納
	int NumFootprint;						//アシアトの個数を記録
	VECTOR Footprint[1000 * 6];				//アシアト機能を実装するための変数

	VECTOR myvecToDxvec(myvector::SVector);
	myvector::SVector dxvecToMyvec(VECTOR Ivector);

	int Rot;								//マウスホイールの回転量初期化

	clock_t startTime, nowTime;						//時間
	unsigned int Cicle_time;

	void DrowTawara( VECTOR center, VECTOR axle, float r1, float r2, float tawaraCoefficient, COLOR_U8 color);

	void ColorDebugMode();

	//描画関数のスケール版　
	void DrawLine3D_s( VECTOR Pos1, VECTOR Pos2, unsigned int Color ) ;
	void DrawTriangle3D_s(  VECTOR Pos1, VECTOR Pos2, VECTOR Pos3, unsigned int Color, int FillFlag );
	void DrawCone3D_s( VECTOR TopPos, VECTOR BottomPos, float r, int DivNum, unsigned int DifColor, unsigned int SpcColor, int FillFlag );
	void DrawCapsule3D_s( VECTOR Pos1, VECTOR Pos2, float r, int DivNum, unsigned int DifColor, unsigned int SpcColor, int FillFlag );
	void DrawSphere3D_s( VECTOR CenterPos, float r, int DivNum, unsigned int DifColor, unsigned int SpcColor, int FillFlag );
	void DrawPolygonIndexed3D_s( VERTEX3D *Vertex, int VertexNum, unsigned short *Indices, int PolygonNum, int GrHandle, int TransFlag );
	void DrowTawara_s( VECTOR center, VECTOR axle, float r1, float r2, float tawaraCoefficient, COLOR_U8 color);
};
