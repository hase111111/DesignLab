#pragma once

#include "vectorFunc.h"
#include "hexapod.h"

#define LEGCOUNT	6						//脚の本数 注意：これを変更しただけで他の脚の本数に対応できるわけでないのであしからず
#define LEGR		20						//脚の半径　2つの脚を　LEGR * 2　以上近づけることはできない
#define HITDETECTIONMARGIN		0.01		//衝突判定のマージン 
#define HITDETECTIONMARGIN2			1.0		//衝突判定のマージン 

class SearchPossibleBodyRotation
{
public:

	//メンバ変数
	Hexapod phantomX;
	myvector::SVector Leg_move[6][20];
	double C_thPRY[3][20];
	
	int pass_body_rotation(int groundLeg[6]);						//胴体を回転させる（ヨー軸のみ）
	//メンバ関数
	SearchPossibleBodyRotation(void);
	~SearchPossibleBodyRotation(void);

	int PossibleLegPoint_Rotation();

private:
	//メンバ変数

	//メンバ関数
	myvector::SVector VCangeBodyToLeg(myvector::SVector& Vin, double thP, double thR, double thY);	//胴体の回転に合わせてVinを回転
};