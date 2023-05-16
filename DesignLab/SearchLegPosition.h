#pragma once

#include "vectorFunc.h"
#include "hexapod.h"
#include "mainfunction.h"
#include "HexapodConst.h"
#include "LegState.h"
#include <map>

							

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

	//PossibleLegPoint_Rotation関数でのみ変更される．脚接地点1,2,3,4,5,6,7 格納されている整数は範囲内の脚接地可能点の数 配列のindexは-1して使用
	int LegGroundablePointNum[HexapodConst::LEG_NUM][LegState::DISCRETE_NUM];		
	//std::map<std::pair<int, int>, int> m_leg_groundable_pos_num;

	//脚接地候補点　候補点の数はlegGroundablePointにて参照 候補点の最大数はLegGroundableCandidatePoint_MAX
	myvector::SVector p_LegGroundableCandidatePoint[HexapodConst::LEG_NUM][LegState::DISCRETE_NUM][1000];
	int LegGroundableCandidatePoint_MAX;

	//メンバ関数	

	//旋回時の脚位置の候補を求める．
	int PossibleLegPoint_Rotation();

	//目標重心高さ変化量計算用．将来的に必要になりそうな重心高さの変更量を返す。
	int Target_delta_comz() const;					

	//胴体と地形（脚接地可能点）との衝突判定。高さ方向に設定したクリアランス分だけ余裕分がないときは、衝突とみなす。Collision_judgment_with_the_body
	int checkBodyMapCollision() const; 

	//すねと地形の干渉チェック 基本的に重心平行移動時用 hato 実機実験用の簡易的な奴．すねと地形（脚接地可能点の一部）の衝突判定
	bool Collision_judgment_with_shin();

private:

	bool m_is_ground[HexapodConst::LEG_NUM];		//脚ごとに遊脚しているかどうかを代入する．旧名 v
	int m_leg_state[HexapodConst::LEG_NUM];		//脚状態を入力する．旧名 kaisou

	//4角形ポリゴンと線分の干渉判定。(ただしポリゴンの頂点4点は同一平面にあるとしてる)
	//4角形ポリゴンと線分の衝突判定 干渉しているとき1,干渉していないとき0 (ただしポリゴンの4頂点同一平面上にある場合)	//ポリゴン3点,線始点,終点
	//Collision_judgment_with_shin()で使用
	bool CollisionDetection_PL4(const myvector::SVector &In1, const myvector::SVector &In2, const myvector::SVector &In3, const myvector::SVector &In4, const myvector::SVector &LineStart, const myvector::SVector &LineEnd);

	//線分と線分の最短距離を求める一応前二つが地形ポリゴンの座標、あと二つが膝と足先の座標で使う予定。
	//calcPointSegmentDistで使用
	double calcPointLineDist(const myvector::SVector &p, const myvector::SVector &In1, const myvector::SVector &V1, myvector::SVector &h, double &t);

	//点と線分の最短距離　p:点 s:線分の始点 e:線分の終点 h:線分上の最短距離になる点 t:媒介変数
	//calcPointSegmentDistで使用
	bool isSharpAngle(const myvector::SVector &p1, const myvector::SVector &p2, const myvector::SVector &p3);
	
	//calcSegmentSegmentDistで使用
	double calcPointSegmentDist(const myvector::SVector &p, const myvector::SVector &s, const myvector::SVector &e, myvector::SVector &h, double t);
	
	//calcSegmentSegmentDistで使用
	double calcLineLineDist(const myvector::SVector &In1,const myvector::SVector &LineStart, const myvector::SVector &V1, const myvector::SVector &V2, myvector::SVector &p1, myvector::SVector &p2, double &t1, double &t2);
	
	//線分と線分の最短距離を求める一応前二つが地形ポリゴンの座標、あと二つが膝と足先の座標で使う予定。返り値は2線分間の最短距離[mm]
	//Collision_judgment_with_shin()で使用
	double calcSegmentSegmentDist(const myvector::SVector &In1, const myvector::SVector &In2, const myvector::SVector &LineStart, const myvector::SVector &LineEnd);

	//他の脚との干渉確認
	//LegGroundableCandidatePointsort_Th
	void checkLegCross(myvector::SVector p_LegGroundableCandidatePoint[6][LegState::DISCRETE_NUM][1000], int LegGroundablePointNum[6][LegState::DISCRETE_NUM]);

	//ソートというか大きいやつ１つわかればいい
	void LegGroundableCandidatePointsort_Th(myvector::SVector p_LegGroundableCandidatePoint[HexapodConst::LEG_NUM][LegState::DISCRETE_NUM][1000], double sort_p_LGCP[HexapodConst::LEG_NUM][LegState::DISCRETE_NUM][1000], int LegGroundablePointNum[HexapodConst::LEG_NUM][LegState::DISCRETE_NUM]);

	//線分が交差しているか判定
	bool isCross(const myvector::SVector& s1, const myvector::SVector& e1, const myvector::SVector& s2, const  myvector::SVector& e2) const;

	bool individualcCheckLegCross(myvector::SVector* LegGroundableCandidatePoint, int* LegGroundablePointNum, int checkedLegNum,int checkLegNum1, int checkLegNum2);

	//グローバル座標→ロボット脚座標(hexapodクラス) の座標変換を行う暮らし
	myvector::SVector VCangeBodyToLeg(const myvector::SVector& Vin) const;

public:
	// getterとsetterをまとめておく
	void SetLegGroundableCandidatePointMAX(const int LGCP_MAX);						//各脚接地候補点の最大数

	// m_is_ground m_leg_stateを初期化する
	void setLegState(const int _leg_state);

	// m_leg_stateの値を取得する
	int getLegState(const int _leg_num) const { return m_leg_state[_leg_num]; }

	// m_is_groundの値を取得する
	bool isGround(const int _leg_num) const { return m_is_ground[_leg_num]; }
};

