#pragma once

#include "vectorFunc.h"
#include "hexapod.h"
#include "Target.h"


//グラフ構造のためのノード(頂点)．旧名 LNODE
struct SNode
{
	int leg_state;									// 脚状態，重心パターンをbitで表す．LegState.hにこれを操作する用の関数がある．旧名 leg_con
	myvector::SVector Leg[HexapodConst::LEG_NUM];	//coxaからの脚位置
	myvector::SVector Leg2[HexapodConst::LEG_NUM];	//coxaからの脚位置
	myvector::SVector global_center_of_mass;		//グローバル重心位置．旧名 GCOM
	float pitch, roll, yaw;						//ピッチ ロール ヨー　Y方向水平が0．旧名 ThP ThR ThY

	struct SNode* parent;							//親ノードへのポインタ
	short node_height;								//葉ノードならば1，1回前葉ノードならば2

	//現在運動履歴として使用，前回の脚上下ノード(上下運動をした場合)2桁，前回の動作1桁，前々回の動作1桁
	int debug;										//この変数はかなり重要なのだが，説明がない!


	float delta_comz;			//重心の上下動作で実際に動いた量[mm]．z座標は上が正
	float target_delta_comz;	//重心の上下動作で動いてほしい量
	int	last_node_num;
	float time;

};

// int   … 4 byte
// float … 4 byte
// double… 8 byte
// char  … 1 byte
// short … 2 byte
// bool  … 1 byte

//ルートの深さを返す関数．
int GetRouteDepth(SNode* My_p);

//ルートの経路を返す再帰的関数．
int GetRoute(SNode* My_p, SNode** Root2My);

// ノードが等しいならtrue.
bool isNodeEqual(const SNode &node1, const SNode &node2);

// ノードを初期化する関数．変更するノードを参照渡しで受け取り，第2引数でランダムに変更するかどうか決める．
void initNode(SNode& _node, const bool _do_random);
