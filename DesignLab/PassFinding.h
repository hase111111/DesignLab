#pragma once

#include "pch.h"
#include "CreateComCandidate.h"
#include "SearchLegPosition.h"
#include "SearchBodyRotation.h"
#include "hexapod.h"
#include "listFunc.h"
#include "BFSinHierarchy.h"
#include "Define.h"

//#define LEGCOUNT	6						//脚の本数
#define EDGE_NUM_2G 5 //重心の上下移動のエッジの数
void init_no_use(std::string filename);


//グラフ探索をどのモードで行うか表した列挙子
enum class ESearchMode
{
	//今使用されているものは 5 だがそれ以外がまともに動くかは定かではない
	// 先輩の残した説明↓
	//m_search_mode=0 2次から探索マルチスレッド用	//m_search_mode=1 4次から探索マルチスレッド用	//m_search_mode=2 4次から探索一般用

	OLDTYPE_ZERO  = 0,	//探索タイプ0:2次方向から直線移動探索,
	OLDTYPE_ONE   = 1,	//1:4次方向から,
	OLDTYPE_TWO   = 2,	//2:シングルスレッド用,ここまで大木さん,
	OLDTYPE_THREE = 3,	//3:2次方向からその場旋回,
	OLDTYPE_FOUR  = 4,	//4:4次方向から,
	EACH_OPERATION = 5,	//5:1動作ごと探索,
	SANE_TIME = 6,		//6:水平移動と上下移動を一度に探索
};

class PassFinding
{
public:
	void operator()();						//マルチスレッド用関数

	//メンバ変数
	LNODE* route;							//動的配列のためのポインタ
	int m_route_max;						//使用するノードの数を取得
	int RET_DIST;							//再度歩き始める時以前の何番目を用いるか
	int DIST_MAX;							//一回の探索で記録するノードの個数

	int CurrentConditionNum;				//ルート探索の初期値の数
	LNODE CurrentCondition[100];			//ルート探索の初期値
	LNODE PastCondition;					//1つ前のノード

	int mapData3D_MAX;						//脚設置可能点の上限
	myvector::SVector* mapData3D;			//脚設置可能点の座標
	std::vector< std::vector< std::vector<myvector::SVector> > > *MapDat;
	int (*pointDataNum)[LP_DIVIDE_NUM];

	LNODE* resultRoute;
	std::ofstream* resultOfstream;
	double *retPriority;	//評価の優先度が1番
	double *retSecond;	//2番目
	double *retThird;
	double *retFourth;

	//メンバ関数
	PassFinding(const int _route_num, std::vector< std::vector< std::vector<myvector::SVector> > > *divideMapData);
	~PassFinding(void);

	void setMapData3D(myvector::SVector *p_mapData3D, int mapData3D_MAX, int pointNum[LP_DIVIDE_NUM][LP_DIVIDE_NUM]);

private:
	//メンバ変数
	int LastNodeNum;						//現在使用しているノード中最大の番号
	int iHX2[2][2][2][2][2][2];
	int HX2[36][Define::LEG_NUM];					//2次方向のグラフ
	int LegHeight;							//脚上げ高さ
	ETargetMode m_evaluation_mode;		//評価をどのように行うか表す変数
	ESearchMode m_search_mode;			//探索をどのように行うか表す変数
	int m_search_depth;					//探索の深さをどうするか決定する変数
	int m_result_route_limit;			//探索するノード数の上限,一度に探索する数が大きくなると変える必要ありCurrentCondition[??]も変える
	int m_node_num;						//探索終了後の結果，ノードの数を送るための関数
	STarget m_target;						//目標
	CreateComCandidate _CreateComCandidate;
	Hexapod phantomX;
	SearchPossibleLegPosition  _SearchPossibleLegPosition;
	SearchPossibleBodyRotation _SearchPossibleBodyRotation;
	int deleteNode(LNODE* PastCondition);

	bool k_no_use[730];



	//メンバ関数
	
	//HX2の初期化
	void initHX2();

	//iHX2の初期化
	void initiHX2();

	//グラフ探索において最初に呼ばれる関数
	//_search_type に基づいて探索を行い， m_search_depth に基づいて探索の深さを決定する
	void getOptimalRoute(ESearchMode _search_type, int m_search_depth);	

	//ノードを列挙する関数．20180327時点で使用している探索方法らしい．旧名 GaitPatternSearch
	void searchGaitPattern();

	//与えられたノードをもとに，遷移可能な階層を探しvector<int>型で参照渡しする．要は遊脚している脚を水平移動させる関数．旧名 pass_transitions_4zi
	void searchTransitionHierarchy(const LNODE node, std::vector<int>& _res_transition_hierarchy);

	//与えられたノードをもとに，遷移可能な重心位置を探しvector<int>型で参照渡しする．要は重心を水平移動させる関数．旧名 pass_transitions_4zi_zyuusinidou
	void searchTransitionCoM(const LNODE node, std::vector<myvector::SLegVector>& _res_leg_pos, std::vector<myvector::SVector>& _res_com_pos, std::vector<int>& _res_state);

	//グラフ探索プログラム
	int chooseMostSuitableSolution(LNODE ret_pass[20]);
	int chooseMostSuitableSolutionRotation(std::ofstream& fout1, LNODE ret_pass[20]);//旋回半径≠0
	int chooseMostSuitableSolutionSpin(std::ofstream& fout1, LNODE ret_pass[20]);		//旋回半径=0

	//与えられた角度を-π～πの範囲で表す
	double limitRangeAngle(const double angle) const;


	int pass_transitions_2zi_zyuusinidou(LNODE* node, int ret_2G_leg_con[5], myvector::SVector ret_2G_leg_add[5], myvector::SVector ret_2G_GCOM_add[5]);
	void pass_transitions_2zi(int v, bool*visited, int ret_2_transition_v[36][6], int* passnum);
	void getGraph(int leg_state, int i_F[LEGCOUNT], int LegGroundablePointNum[LEGCOUNT][DISCRETE_NUM], bool visited[36]);
	void getLegGroundablePoint(LNODE* route, int LegGroundablePointNum[LEGCOUNT][DISCRETE_NUM], myvector::SVector LegGroundablePoint[LEGCOUNT][DISCRETE_NUM][100]);
	void pass_body_rotation(LNODE* node, double ret_thPRY[3][20], myvector::SVector ret_Leg_move[LEGCOUNT][20], int* Rotation_passnum);//胴体回転角度の探索

	int SelectLegPoint_Rotation(LNODE* node, int LegGroundablePointNum[LEGCOUNT][DISCRETE_NUM], myvector::SVector LegGroundablePoint[LEGCOUNT][DISCRETE_NUM][100]);//旋回時の脚位置計算
	int SelectLegPoint_Rotation(const int _leg_num, const LNODE node, int LegGroundablePointNum[LEGCOUNT][DISCRETE_NUM]);	//1脚版
	double Stability_Margin(myvector::SVector leg[6], int v[6]);//安定余裕計算

	//void Yozihoukou_idou();現在使用されていない．大木さんのプログラム参照とのこと
	bool isAbleLegPosComType(LNODE* route);	//未使用関数
	void calculatePassLegRotation();			//旋回時の脚位置計算

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
