#pragma once

#include "pch.h"
#include "vectorFunc.h"
#include "hexapod.h"
#include "Target.h"

//ビット演算用の定数　for分で扱えるよう配列で	190528
constexpr int leg_bit[6] = { (0b1111 << 0),
							 (0b1111 << 4),
							 (0b1111 << 8),
							 (0b1111 << 12),
							 (0b1111 << 16),
							 (0b1111 << 20)
};
constexpr int COM_bit = (0b1111 << 24);
constexpr int shift_leg_bit[6] = { 0, 4, 8, 12, 16, 20 };	//4bitずつ
constexpr int shift_COM = 24;
constexpr int kaisou_bit = 0b0111;	//脚位置を示す部分
constexpr int v_bit = 0b1000;	//vは遊脚の組み合わせ(2次位置)のこと　なぜvかは知らん ←　知っとけ


//グラフ構造のためのNODE．グラフ構造を作るためにどうやらポインタを使っている様子
typedef struct t_node
{
	//360byteだったよ 190422

	int center_of_mass;							//重心位置
	double pitch, roll, yaw;					//ピッチ ロール ヨー　Y方向水平が0
	myvector::SVector Leg[6];					//coxaからの脚位置
	myvector::SVector Leg2[6];					//coxaからの脚位置
	myvector::SVector global_center_of_mass;		//グローバル重心位置
	struct t_node* parent;						//親ノードへのポインタ
	short node_height;							//葉ノードならば1，1回前葉ノードならば2

	//現在運動履歴として使用，前回の脚上下ノード(上下運動をした場合)2桁，前回の動作1桁，前々回の動作1桁
	int debug;									//この変数はかなり重要なのだが，説明がない!

	//脚状態って言ってるけど重心タイプも入ってる．C++だからint型は32bit．	1脚の脚状態を4bitで表す 最上位0:遊脚,1:接地　
	//残り3bitで離散化した脚位置．脚は右前脚を0として時計回りに0~5
	//	7   3    (0は使わない)
	//	6 4 2
	//	5   1
	//1111    1111		1111 1111 1111 1111 1111 1111
	//余り    重心タイプ  脚5  脚4  脚3  脚2  脚1  脚0
	int leg_state;	
	
	double delta_comz;			//重心の上下動作で実際に動いた量[mm]．z座標は上が正
	double target_delta_comz;	//重心の上下動作で動いてほしい量
	int	last_node_num;
	double time;

} LNODE;



int GetRouteDepth(LNODE* My_p);
int GetRoute(LNODE* My_p, LNODE** Root2My);
void LNODEOutPut(LNODE LNODE1);
double AverageLegHeight(LNODE node, myvector::SVector *p_mapData3D, int mapData3D_MAX);

/*
//leg_conと脚番号からその脚が接地しているか返す
//1:接地　0:遊脚
inline bool isGrounded(int cond, int lnum);

//leg_conと脚番号からその脚位置を返す
inline int numOfLegPosi(int cond, int lnum);
//COMTypeを返す
inline int numOfCOMType(int cond);
*/


//leg_conと脚番号からその脚位置を返す	1～7
inline int numOfLegPosi(int cond, int lnum) 
{
	return ((cond & (kaisou_bit << shift_leg_bit[lnum])) >> shift_leg_bit[lnum]);
}

inline int numOfCOMType(int cond) 
{
	return ((cond & COM_bit) >> shift_COM);
}

bool LNODEEqual(const LNODE &node1, const LNODE &node2);