#pragma once
//#include "CreateComCandidate.h"
#include "SearchLegPosition.h"
//#include "SearchBodyRotation.h"
#include "hexapod.h"
#include "Node.h"
#include "BFSinHierarchy.h"
#include "Define.h"
#include "MapConst.h"
#include "ComType.h"

//グラフ探索をどのモードで行うか表した列挙子
enum class ESearchMode
{
	//今使用されているものは 5 だがそれ以外がまともに動くかは定かではない
	// 先輩の残した説明↓
	//m_search_mode=0 2次から探索マルチスレッド用	//m_search_mode=1 4次から探索マルチスレッド用	//m_search_mode=2 4次から探索一般用

	OLDTYPE_ZERO = 0,	//探索タイプ0:2次方向から直線移動探索,
	OLDTYPE_ONE = 1,	//1:4次方向から,
	OLDTYPE_TWO = 2,	//2:シングルスレッド用,ここまで大木さん,
	OLDTYPE_THREE = 3,	//3:2次方向からその場旋回,
	OLDTYPE_FOUR = 4,	//4:4次方向から,
	EACH_OPERATION = 5,	//5:1動作ごと探索,
	SANE_TIME = 6,		//6:水平移動と上下移動を一度に探索
};

class PassFinding final
{
public:
	void operator()();						//マルチスレッド用関数

	//メンバ変数
	SNode* route;							//動的配列のためのポインタ
	int m_route_max;						//使用するノードの数を取得
	int RET_DIST;							//再度歩き始める時以前の何番目を用いるか
	int DIST_MAX;							//一回の探索で記録するノードの個数

	int CurrentConditionNum;				//ルート探索の初期値の数
	SNode CurrentCondition[100];			//ルート探索の初期値

	SNode* resultRoute;
	float* retPriority;	//評価の優先度が1番
	float* retSecond;	//2番目
	float* retThird;
	float* retFourth;

	//メンバ関数
	PassFinding(const int _route_num, const MapState* _p_map_state);
	~PassFinding();

private:
	//メンバ変数
	int LastNodeNum;						//現在使用しているノード中最大の番号
	int HX2[36][HexapodConst::LEG_NUM];					//2次方向のグラフ
	int LegHeight;							//脚上げ高さ
	ETargetMode m_evaluation_mode;		//評価をどのように行うか表す変数
	ESearchMode m_search_mode;			//探索をどのように行うか表す変数
	int m_search_depth;					//探索の深さをどうするか決定する変数
	int m_result_route_limit;			//探索するノード数の上限,一度に探索する数が大きくなると変える必要ありCurrentCondition[??]も変える
	int m_node_num;						//探索終了後の結果，ノードの数を送るための関数
	STarget m_target;						//目標
	//CreateComCandidate _CreateComCandidate;
	Hexapod phantomX;
	SearchPossibleLegPosition  _SearchPossibleLegPosition;
	int deleteNode(SNode* PastCondition);


	//メンバ関数

	//HX2の初期化
	void initHX2();

	//グラフ探索において最初に呼ばれる関数
	//_search_type に基づいて探索を行い， m_search_depth に基づいて探索の深さを決定する
	void getOptimalRoute(ESearchMode _search_type, int m_search_depth);

	//ノードを列挙する関数．20180327時点で使用している探索方法らしい．旧名 GaitPatternSearch
	void searchGaitPattern();

	//与えられたノードをもとに，遷移可能な階層を探しvector<int>型で参照渡しする．要は遊脚している脚を水平移動させる関数．旧名 pass_transitions_4zi
	void searchTransitionHierarchy(const SNode& node, std::vector<int>& _res_transition_hierarchy);

	//与えられたノードをもとに，遷移可能な重心位置を探しvector<int>型で参照渡しする．要は重心を水平移動させる関数．旧名 pass_transitions_4zi_zyuusinidou
	void searchTransitionCoM(const SNode& node, std::vector<my_vec::SLegVector>& _res_leg_pos, std::vector<my_vec::SVector>& _res_com_pos, std::vector<int>& _res_state);

	//与えられたノードをもとに，遷移可能な縦方向の重心の位置をvector<int>型で参照渡しする．要は重心を上下移動させる関数．旧名 pass_transitions_2zi_zyuusinidou
	// (過去の説明) 胴体の上下移動のエッジ．現在の体勢での可動限界を返す．重心高さはロボット座標系
	int searchTransitionComVertical(const SNode& _node, const int _devide_num, my_vec::SVector ret_2G_leg_add[5], my_vec::SVector ret_2G_GCOM_add[5]);

	//グラフ探索プログラム
	int chooseMostSuitableSolution(SNode ret_pass[20]);
	int chooseMostSuitableSolutionRotation(SNode ret_pass[20]);//旋回半径≠0
	int chooseMostSuitableSolutionSpin(SNode ret_pass[20]);		//旋回半径=0

	//与えられた角度を-π～πの範囲で表す
	float limitRangeAngle(const float angle) const;


	void pass_transitions_2zi(int v, bool* visited, int ret_2_transition_v[ComType::COM_TYPE_NUM][6], int* passnum);
	void getGraph(const int leg_state, bool i_F[HexapodConst::LEG_NUM], int LegGroundablePointNum[HexapodConst::LEG_NUM][LegStateEdit::DISCRETE_NUM], bool visited[ComType::COM_TYPE_NUM]);
	void getLegGroundablePoint(SNode* route, int LegGroundablePointNum[HexapodConst::LEG_NUM][LegStateEdit::DISCRETE_NUM], my_vec::SVector LegGroundablePoint[HexapodConst::LEG_NUM][LegStateEdit::DISCRETE_NUM][100]);

	int SelectLegPoint_Rotation(const SNode& node, int LegGroundablePointNum[HexapodConst::LEG_NUM][LegStateEdit::DISCRETE_NUM], my_vec::SVector LegGroundablePoint[HexapodConst::LEG_NUM][LegStateEdit::DISCRETE_NUM][100]);//旋回時の脚位置計算

	float Stability_Margin(my_vec::SVector leg[6], int v[6]);//安定余裕計算

public:
	// Getter と Setter まとめ，"Tell. Don't Ask." なプログラムにしたいので余り健全ではない 

	//評価方法を設定する関数
	void setEvaluationMode(const ETargetMode _mode) { m_evaluation_mode = _mode; }
	//探索方法を設定する関数
	void setSearchMode(const ESearchMode _mode) { m_search_mode = _mode; }
	//探索の深さを設定する関数
	void setSearchDepth(const int _depth) { m_search_depth = _depth; }
	//探索するノード数の上限を設定する関数
	void setResultRouteLimit(const int _limit) { m_result_route_limit = _limit; }
	//探索の目標値を設定する関数
	void setTarget(const STarget _target) { m_target = _target; }

	//探索に使用したノードの数を取得する関数
	int getNodeNum() const { return m_node_num; }
};
