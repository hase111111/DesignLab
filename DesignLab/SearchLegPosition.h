#pragma once

#include "vectorFunc.h"
#include "hexapod.h"
#include "mainfunction.h"
#include "HexapodConst.h"

#define LEGR		20						//脚の半径　2つの脚を　LEGR * 2　以上近づけることはできない
#define HITDETECTIONMARGIN		0.01		//衝突判定のマージン 
#define HITDETECTIONMARGIN2			1.0		//衝突判定のマージン 

#define DISCRETE_NUM 7	//脚位置の離散化数

//胴体重心と脚接地可能点との衝突マージン．hexapod.hのMIN_DELTAZの値と合わせる
#define BODY_MARGIN		HexapodConst::VERTICAL_MIN_RANGE	
							

//面を構成する4つの頂点座標は、mainfunction.cppで定義してる。

//#define COLLISION_SHIN1	//1つ目の面との脛の干渉チェックをするときはコメントアウトをはずす

#define COLLISION_SHIN2		//2つ目の面との脛の干渉チェックをするときはコメントアウトすをはずす

//#define COLLISION_SHIN3	//3つ目の面との脛の干渉チェックをするときはコメントアウトをはずす
							


#define SHIN_R	20			//脛の半径[mm]、カプセルで近似している。

#define SHIN_L	75			//脛を表すカプセルの長さで、実際の脛の長さとの比率(%)、
							//75くらいがちょうどいい。鉛直面との干渉だけチェックするなら80でもいい（確か）。水平面も入れると、足が地面に対して斜めった状態でも干渉してる判定になるから。
							//Collision_judgment_with_shin()を使用しているのは、いまのところ重心の平行移動(CreateComCandidate.h)のところだけ2020/11/09hato

class SearchPossibleLegPosition
{
public:
	SearchPossibleLegPosition();
	~SearchPossibleLegPosition() = default;

	//メンバ変数
	Hexapod phantomX;
	myvector::SVector* p_mapData3D;						//脚接地可能点
	std::vector< std::vector< std::vector<myvector::SVector> > > *sMapData;	//3次元vector 平方分割された状態の脚接地可能点
	int(*spointDataNum)[LP_DIVIDE_NUM];
	int mapData3D_MAX;							//脚接地可能点の最大数

	int LegGroundablePointNum[HexapodConst::LEG_NUM][DISCRETE_NUM];		//脚接地点1,2,3,4,5,6,7 格納されている整数は範囲内の脚接地可能点の数 配列のindexは-1して使用

	int v[HexapodConst::LEG_NUM];
	int kaisou[HexapodConst::LEG_NUM];	//追加hato

	//脚接地候補点　候補点の数はlegGroundablePointにて参照 候補点の最大数はLegGroundableCandidatePoint_MAX
	myvector::SVector p_LegGroundableCandidatePoint[HexapodConst::LEG_NUM][DISCRETE_NUM][1000];			
	int LegGroundableCandidatePoint_MAX;

	//メンバ関数
	void LegGroundableCandidatePointsort_Th(myvector::SVector p_LegGroundableCandidatePoint[HexapodConst::LEG_NUM][DISCRETE_NUM][1000], double sort_p_LGCP[HexapodConst::LEG_NUM][DISCRETE_NUM][1000], int LegGroundablePointNum[HexapodConst::LEG_NUM][DISCRETE_NUM]);
	void LegGroundableCandidatePointsort_Th(int legnum, myvector::SVector p_LegGroundableCandidatePoint[HexapodConst::LEG_NUM][DISCRETE_NUM][1000], double sort_p_LGCP[DISCRETE_NUM][1000], int LegGroundablePointNum[HexapodConst::LEG_NUM][DISCRETE_NUM]);	//1脚版

	//もろもろ初期値を与えた後実行　返り値－1でエラー
	int calculateLegGroundablePoint();			

	int PossibleLegPoint_Rotation();

	//1脚版
	int PossibleLegPoint_Rotation(int legnum);	

	//目標重心高さ変化量計算用
	int Target_delta_comz();					

	int Collision_judgment_with_the_body(); //胴体と脚設置可能点の衝突判定()重心上下用
	bool Collision_judgment_with_shin();	//すねと地形（脚接地可能点の一部）の衝突判定()基本的に重心平行移動時用 hato 実機実験用の簡易的な奴
	myvector::SVector VCangeBodyToLeg(myvector::SVector &Vin);

private:

	//指定した点PがベクトルSMとベクトルMEの張る面に対して表にあるか裏にあるか判定SM×MEのベクトルの指す方向が表
	bool Check_front_and_back(const myvector::SVector& S, const myvector::SVector& M, const myvector::SVector& E, const myvector::SVector& P);

	//三角形ポリゴンと線分の衝突判定 接する場合衝突としない  ポリゴン3点,線始点,終点
	bool	CollisionDetection_PL3(const myvector::SVector &In1, const myvector::SVector &In2, const myvector::SVector &In3, const myvector::SVector &LineStart, const myvector::SVector &LineEnd);

	//4角形ポリゴンと線分の干渉判定。(ただしポリゴンの頂点4点は同一平面にあるとしてる)
	bool	CollisionDetection_PL4(const myvector::SVector &In1, const myvector::SVector &In2, const myvector::SVector &In3, const myvector::SVector &In4, const myvector::SVector &LineStart, const myvector::SVector &LineEnd);

	//線分と線分の最短距離を求める一応前二つが地形ポリゴンの座標、あと二つが膝と足先の座標で使う予定。
	double calcPointLineDist(const myvector::SVector &p, const myvector::SVector &In1, const myvector::SVector &V1, myvector::SVector &h, double &t);

	bool isSharpAngle(const myvector::SVector &p1, const myvector::SVector &p2, const myvector::SVector &p3);
	double calcPointSegmentDist(const myvector::SVector &p, const myvector::SVector &s, const myvector::SVector &e, myvector::SVector &h, double t);
	double calcLineLineDist(const myvector::SVector &In1,const myvector::SVector &LineStart, const myvector::SVector &V1, const myvector::SVector &V2, myvector::SVector &p1, myvector::SVector &p2, double &t1, double &t2);
	double calcSegmentSegmentDist(const myvector::SVector &In1, const myvector::SVector &In2, const myvector::SVector &LineStart, const myvector::SVector &LineEnd);

	//LegGroundableCandidatePointを進行方向順にソート
	void LegGroundableCandidatePointsort(myvector::SVector p_LegGroundableCandidatePoint[6][DISCRETE_NUM][1000], int LegGroundablePointNum[6][DISCRETE_NUM]);

	//他の脚との干渉確認
	void checkLegCross(myvector::SVector p_LegGroundableCandidatePoint[6][DISCRETE_NUM][1000], int LegGroundablePointNum[6][DISCRETE_NUM]);

	bool isCross(myvector::SVector s1, myvector::SVector e1, myvector::SVector s2, myvector::SVector e2);

	bool ContactJudgment(double t1, double t2);

	bool VectorEqual(myvector::SVector v1, myvector::SVector v2);

	bool individualcCheckLegCross(myvector::SVector* LegGroundableCandidatePoint, int* LegGroundablePointNum, int checkedLegNum,int checkLegNum1, int checkLegNum2);

public:
	// getterとsetterをまとめておく
	void SetLegGroundableCandidatePointMAX(const int LGCP_MAX);						//各脚接地候補点の最大数
};

