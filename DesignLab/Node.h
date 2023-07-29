#pragma once
#include "MyVector.h"
#include "Target.h"
#include "HexapodNextMove.h"
#include <string>
#include <iostream>


struct SNode
{
	SNode();
	SNode(const SNode& _other);

	//! @brief 初期化関数
	//! @param [in] _random 初期位置をランダムにするかどうか
	void init(const bool _random);

	//! @brief 重心位置を変更する関数．
	//! @param [in] new_com 新しい重心位置
	//! @param [in] do_change_leg_base_pos 遊脚中の脚の接地基準地点の座標を変更するかどうか
	//! @details 脚位置は脚の付け根からの相対座標で表現されている． <br>
	//! 遊脚している脚は一緒に移動するが，接地脚は移動しないため座標を変更してやる必要がある．
	void changeGlobalCenterOfMass(const my_vec::SVector& new_com, const bool do_change_leg_base_pos);

	//! @brief 自身を親ノードに変更する関数．
	//! @details depthを0に，parent_numを-1に初期化する．
	constexpr void changeParentNode()
	{
		depth = 0;
		parent_num = -1;
	}

	//! @brief 次の動作を設定する関数．
	//! @param [in] _parent_num 親ノードの番号
	//! @param [in] _next_move 次の動作
	//! @details 深さを一つ深くして，親と次の動作を設定する．
	constexpr void changeNextNode(const int _parent_num, const EHexapodMove _next_move)
	{
		depth++;
		parent_num = _parent_num;
		next_move = _next_move;
	}

	//! 代入演算子
	constexpr SNode& operator=(const SNode& _other)
	{
		leg_state = _other.leg_state;

		for (int i = 0; i < HexapodConst::LEG_NUM; i++)
		{
			leg_pos[i] = _other.leg_pos[i];
			leg_base_pos[i] = _other.leg_base_pos[i];
		}

		global_center_of_mass = _other.global_center_of_mass;
		rot = _other.rot;

		next_move = _other.next_move;
		parent_num = _other.parent_num;
		depth = _other.depth;

		return *this;
	}

	//! 比較演算子
	constexpr bool operator == (const SNode& _other) const
	{
		if (next_move != _other.next_move) { return false; }

		if (leg_state != _other.leg_state) { return false; }

		for (int i = 0; i < HexapodConst::LEG_NUM; i++)
		{
			if (leg_pos[i] != _other.leg_pos[i]) { return false; }
			//if (leg_base_pos[i] != _other.leg_base_pos[i]) { return false; }
		}

		if (global_center_of_mass != _other.global_center_of_mass) { return false; }
		if (rot != _other.rot) { return false; }

		return true;
	}

	constexpr bool operator != (const SNode& _other) { return !(*this == _other); }

	// 4 + 72 + 72 + 12 + 12 + 1 + 4 + 1 = 176 byte

	int leg_state;										//!< [4 byte] 脚状態，重心パターンをbitで表す．LegState.hにこれを操作する用の名前空間と関数がある．旧名 leg_con

	my_vec::SVector leg_pos[HexapodConst::LEG_NUM];		//!< [4 * 3 * 6 = 72 byte] 脚先の座標．(coxa(脚の付け根)を原点とする)
	my_vec::SVector leg_base_pos[HexapodConst::LEG_NUM];//!< [4 * 3 * 6 = 72 byte] 脚接地の基準点の座標．離散化した時に4になる地点．(coxa(脚の付け根)を原点とする)
	my_vec::SVector global_center_of_mass;				//!< [4 * 3 = 12byte] グローバル座標における重心の位置．旧名 GCOM
	my_vec::SRotator rot;								//!< [4 * 3 = 12byte] ロール(X軸) ピッチ(Y軸) ヨー(Z軸)　右ねじの方向を正回転とし，ロボットの重心を原点に回転する．旧名 ThP ThR ThY

	EHexapodMove next_move;		//!< [1 byte] 次の動作を代入する．元のプログラムではint debugが担っていた仕事を行う．
	int parent_num;				//!< [4 byte] 自身の親がvector配列のどこにいるのかを記録する．親がいないなら負の値をとる．
	char depth;					//!< [1 byte] 自身の深さ．一番上の親が深さ0となる．おそらく128を超えるような値をとらないと思うので一番小さなchar型を使用する．
};

std::ostream& operator<<(std::ostream& stream, const SNode& value);


//! @file Node.h
//! @brief ノードの構造体を定義する．
//! @details ロボットの座標系は通例，進行方向をXの正，ロボットの真上をZの正，y軸は右手座標系でとるらしい．このプログラムもそのように統一する．<br>
//! 過去のプログラムではそれがバラバラになっていたため，途中で座標系を変換する処理が多々あったが，煩雑なうえ，時間がかかるので，全て統一する．<br>
//! ポインタ  8 byte <br> int   … 4 byte <br> float … 4 byte <br> double… 8 byte <br> char  … 1 byte <br> short … 2 byte <br> bool  … 1 byte
//! @date 2023/07/14
//! @auther 長谷川 

//! //! @struct SNode
//! @brief グラフ構造のためのノード(頂点)．旧名 LNODE
//! @details この構造体は，グラフ構造のためのノード(頂点)である． <br>
//! 先行研究のプログラムに比べて大きく内容を変更したが，基本的な構造は同じである． <br>
//! @date 2023/07/19
//! @auther 長谷川