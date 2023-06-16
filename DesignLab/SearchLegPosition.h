#pragma once
#include "vectorFunc.h"
#include "hexapod.h"
#include "MapConst.h"
#include "HexapodConst.h"
#include "LegState.h"
#include "MapState.h"
#include <map>							

//このクラスはかつてはCCCのメンバでもあったが，機能が使用されていなかったので取り除いた．
//かつて使用されていた関数はファイルの末尾に示す．

class SearchPossibleLegPosition final
{
public:
	SearchPossibleLegPosition();
	~SearchPossibleLegPosition() = default;

	//メンバ変数
	Hexapod phantomX;

	//マップの状態を受け取るためのポインタ．
	const MapState* mp_MapState;

	//PossibleLegPoint_Rotation関数でのみ変更される．脚接地点1,2,3,4,5,6,7 格納されている整数は範囲内の脚接地可能点の数 配列のindexは-1して使用
	int LegGroundablePointNum[HexapodConst::LEG_NUM][leg_state::DISCRETE_NUM];		
	//std::map<std::pair<int, int>, int> m_leg_groundable_pos_num;

	//脚接地候補点　候補点の数はlegGroundablePointにて参照 候補点の最大数はLegGroundableCandidatePoint_MAX
	myvector::SVector p_LegGroundableCandidatePoint[HexapodConst::LEG_NUM][leg_state::DISCRETE_NUM][1000];
	int LegGroundableCandidatePoint_MAX;

	//メンバ関数	

	//旋回時の脚位置の候補を求める．
	int PossibleLegPoint_Rotation();

	//目標重心高さ変化量計算用．将来的に必要になりそうな重心高さの変更量を返す。
	int Target_delta_comz() const;					

	//胴体と地形（脚接地可能点）と衝突していないか判定し，衝突しないためにはどれだけ移動すればよいか返す．移動する必要がないならば０を返す．旧名Collision_judgment_with_the_body
	float calculateCollisionAvoidanceMovement() const; 

private:

	bool m_is_ground[HexapodConst::LEG_NUM];	//脚ごとに遊脚しているかどうかを代入する．旧名 v
	int m_leg_state[HexapodConst::LEG_NUM];		//脚状態を入力する．旧名 kaisou


	//他の脚との干渉確認
	//LegGroundableCandidatePointsort_Thで使用
	void checkLegCross(myvector::SVector p_LegGroundableCandidatePoint[6][leg_state::DISCRETE_NUM][1000], int LegGroundablePointNum[6][leg_state::DISCRETE_NUM]);

	//ソートというか大きいやつ１つわかればいい
	void LegGroundableCandidatePointsort_Th(myvector::SVector p_LegGroundableCandidatePoint[HexapodConst::LEG_NUM][leg_state::DISCRETE_NUM][1000], float sort_p_LGCP[HexapodConst::LEG_NUM][leg_state::DISCRETE_NUM][1000], int LegGroundablePointNum[HexapodConst::LEG_NUM][leg_state::DISCRETE_NUM]);

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

//定義
//#define SHIN_L	75			//脛を表すカプセルの長さで、実際の脛の長さとの比率(%)、
//#define SHIN_R	20			//脛の半径[mm]、カプセルで近似している。
//
//メンバ関数
//public:
//		bool Collision_judgment_with_shin();
//private:
//	bool CollisionDetection_PL4
//	float calcPointLineDist
//	bool isSharpAngle
//	float calcPointSegmentDist
//	float calcLineLineDist
//	float calcSegmentSegmentDist
//
//以上．結構ミスも多いように感じたので使いたいなら修正必須です．