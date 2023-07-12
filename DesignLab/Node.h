#pragma once
#include "MyVector.h"
#include "hexapod.h"
#include "Target.h"
#include <string>

//ロボットの座標系は通例，進行方向をXの正，ロボットの真上をZの正，y軸は右手座標系でとるらしい．このプログラムもそのように統一する．
//過去のプログラムではそれがバラバラになっていたため，途中で座標系を変換する処理が多々あったが，煩雑なうえ，時間がかかるので，全て統一する．


//ロボットが次にどの動作をするのかを表す列挙体．サイズは 2 byte．sizeofで確認済み
enum class EHexapodMove : char
{
	NONE = 0,					// 何も動作をしない
	LEG_UP_DOWN = 1,			// 脚の上下移動.
	LEG_HIERARCHY_CHANGE = 2,	// 脚の平行移動．脚の階層を変更する．
	COM_MOVE = 3,				// 重心の平行移動．Center Of Massで重心のこと．
	COM_UP_DOWN = 4,			// 重心の上下移動
};


//グラフ構造のためのノード(頂点)．旧名 LNODE
struct SNode
{
	SNode();
	SNode(const SNode& _other)
	{
		leg_state = _other.leg_state;

		for (int i = 0; i < HexapodConst::LEG_NUM; i++)
		{
			leg_pos[i] = _other.leg_pos[i];
			Leg2[i] = _other.Leg2[i];
		}

		global_center_of_mass = _other.global_center_of_mass;
		rot = _other.rot;

		next_move = _other.next_move;
		parent_num = _other.parent_num;
		depth = _other.depth;


		//現在未使用の変数．後で消します
		parent = _other.parent;
		node_height = _other.node_height;
		debug = _other.debug;
		last_node_num = _other.last_node_num;
		time = _other.time;
		roll = _other.roll;
		pitch = _other.pitch;
		yaw = _other.yaw;
		delta_comz = _other.delta_comz;
		target_delta_comz = _other.target_delta_comz;
	}

	//! @brief 初期化関数
	//! @param[in] _random 初期位置をランダムにするかどうか
	void init(const bool _random);

	//! 代入演算子
	constexpr SNode& operator=(const SNode& _other)
	{
		leg_state = _other.leg_state;

		for (int i = 0; i < HexapodConst::LEG_NUM; i++)
		{
			leg_pos[i] = _other.leg_pos[i];
			Leg2[i] = _other.Leg2[i];
		}

		global_center_of_mass = _other.global_center_of_mass;
		rot = _other.rot;

		next_move = _other.next_move;
		parent_num = _other.parent_num;
		depth = _other.depth;


		//現在未使用の変数．後で消します
		parent = _other.parent;				//親ノードのポインタ
		node_height = _other.node_height;	//ノード高さ
		debug = _other.debug;				//現在運動履歴として使用,前回の脚上下ノード(上下運動をした場合)2桁,前回の動作1桁,前々回の動作1桁
		last_node_num = _other.last_node_num;
		time = _other.time;
		roll = _other.roll;
		pitch = _other.pitch;
		yaw = _other.yaw;
		delta_comz = _other.delta_comz;
		target_delta_comz = _other.target_delta_comz;

		return *this;
	}

	//! 比較演算子
	constexpr bool operator == (const SNode& _other)
	{
		if (leg_state != _other.leg_state) { return false; }

		for (int i = 0; i < HexapodConst::LEG_NUM; i++)
		{
			if (leg_pos[i] != _other.leg_pos[i]) { return false; }
			if (Leg2[i] != _other.Leg2[i]) { return false; }
		}

		if (global_center_of_mass != _other.global_center_of_mass) { return false; }
		if (rot != _other.rot) { return false; }

		return true;
	}
	constexpr bool operator != (const SNode& _other) { return !(*this == _other); }

	// 4 + 72 + 72 + 12 + 12 + 1 + 4 + 1

	int leg_state;									//!< [4 byte] 脚状態，重心パターンをbitで表す．LegState.hにこれを操作する用の名前空間と関数がある．旧名 leg_con

	my_vec::SVector leg_pos[HexapodConst::LEG_NUM];	//!< [4 * 3 * 6 = 72 byte] coxa(脚の付け根)を原点とした脚先の位置
	my_vec::SVector Leg2[HexapodConst::LEG_NUM];	//!< [4 * 3 * 6 = 72 byte] coxa(脚の付け根)を原点とした，脚の離散化をした時，4となる脚先の位置
	my_vec::SVector global_center_of_mass;			//!< [4 * 3 = 12byte] グローバル座標における重心の位置．旧名 GCOM
	my_vec::SRotator rot;							//!< [4 * 3 = 12byte] ロール(X軸) ピッチ(Y軸) ヨー(Z軸)　右ねじの方向を正回転とし，ロボットの重心を原点に回転する．旧名 ThP ThR ThY

	EHexapodMove next_move;		//!< [1 byte] 次の動作を代入する．元のプログラムではint debugが担っていた仕事を行う．
	int parent_num;				//!< [4 byte] 自身の親がvector配列のどこにいるのかを記録する．親がいないなら負の値をとる．
	char depth;					//!< [1 byte] 自身の深さ．一番上の親が深さ0となる．おそらく128を超えるような値をとらないと思うので一番小さなchar型を使用する．

	//以下未使用の変数．後で削除する
	struct SNode* parent;		//親ノードへのポインタ
	short node_height;			//葉ノードならば1，1回前葉ノードならば2
	int debug;					//現在運動履歴として使用，前回の脚上下ノード(上下運動をした場合)2桁，前回の動作1桁，前々回の動作1桁
	int	last_node_num;
	float time;
	float roll, pitch, yaw;
	float delta_comz;			//重心の上下動作で実際に動いた量[mm]．z座標は上が正
	float target_delta_comz;	//重心の上下動作で動いてほしい量
};

//ポインタ  8 byte 
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
bool isNodeEqual(const SNode& node1, const SNode& node2);

