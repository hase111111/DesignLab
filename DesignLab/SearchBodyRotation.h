#pragma once

#include "vectorFunc.h"
#include "hexapod.h"

//このクラスは現在のプログラムでは全く使用されていないため，全ての関数をコメントアウトしてある．
//もともとはPassFindingのメンバだった．

class SearchPossibleBodyRotation
{
public:

	//メンバ変数
	Hexapod phantomX;
	myvector::SVector Leg_move[6][20];
	double C_thPRY[3][20];

	//メンバ関数
	int pass_body_rotation(int groundLeg[6]);						//胴体を回転させる（ヨー軸のみ）

	SearchPossibleBodyRotation();
	~SearchPossibleBodyRotation() = default;

	int PossibleLegPoint_Rotation();

private:
	//メンバ変数

	//メンバ関数
	myvector::SVector VCangeBodyToLeg(const myvector::SVector& Vin, const double thP, const double thR, const double thY) const;	//胴体の回転に合わせてVinを回転
};