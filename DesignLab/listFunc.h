#pragma once

#include "vectorFunc.h"
#include "hexapod.h"
#include "Target.h"


//グラフ構造のためのNODE．グラフ構造を作るためにどうやらポインタを使っている様子
typedef struct t_node
{
	//360byteだったよ 190422

	int center_of_mass;							//重心位置
	double pitch, roll, yaw;					//ピッチ ロール ヨー　Y方向水平が0
	myvector::SVector Leg[6];					//coxaからの脚位置
	myvector::SVector Leg2[6];					//coxaからの脚位置
	myvector::SVector global_center_of_mass;	//グローバル重心位置
	struct t_node* parent;						//親ノードへのポインタ
	short node_height;							//葉ノードならば1，1回前葉ノードならば2

	//現在運動履歴として使用，前回の脚上下ノード(上下運動をした場合)2桁，前回の動作1桁，前々回の動作1桁
	int debug;									//この変数はかなり重要なのだが，説明がない!

	// LegState.hにこれを操作する用の関数がある
	int leg_state;	
	
	double delta_comz;			//重心の上下動作で実際に動いた量[mm]．z座標は上が正
	double target_delta_comz;	//重心の上下動作で動いてほしい量
	int	last_node_num;
	double time;

} LNODE;

//ルートの深さを返す関数．
int GetRouteDepth(LNODE* My_p);

//ルートの経路を返す再帰的関数．
int GetRoute(LNODE* My_p, LNODE** Root2My);

// ノードが等しいならtrue.
bool isLNODEEqual(const LNODE &node1, const LNODE &node2);

// ノードを初期化する関数．変更するノードを参照渡しで受け取り，第2引数でランダムに変更するかどうか決める．
void initLNODE(LNODE& _node, const bool _do_random);
