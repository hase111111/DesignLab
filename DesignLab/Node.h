//! @file node.h
//! @brief ノードの構造体を定義する．
//! @details ロボットの座標系は通例，進行方向をXの正，ロボットの真上をZの正，y軸は右手座標系でとるらしい．
//! @n このプログラムもそのように統一する．
//! @n 過去のプログラムではそれがバラバラになっていたため，途中で座標系を変換する処理が多々あったが，
//! @n 煩雑なうえ，時間がかかるので，全て統一する．
//! @n 
//! @n [参考] 
//! @n ポインタ  8 byte 
//! @n int   … 4 byte 
//! @n float … 4 byte 
//! @n double… 8 byte 
//! @n char  … 1 byte 
//! @n short … 2 byte 
//! @n bool  … 1 byte
//! @n (1byte = 8bit)
//! @n
//! @n 軽くするためにintではなくshortやcharを使うのはやめたほうが良い，
//! @n なぜなら，計算処理の問題で，intよりも小さい型はintに変換されてから計算されるため，計算時間がかかるからである．(嘘松かも)

#pragma once

#include <array>
#include <iostream>
#include <string>

#include "designlab_vector3.h"
#include "designlab_euler.h"
#include "hexapod_next_move.h"
#include "leg_state.h"
#include "target.h"


//! @struct SNode
//! @brief グラフ構造のためのノード(頂点)．旧名 LNODE
//! @details この構造体は，グラフ構造のためのノード(頂点)である． 
//! @n ノードは，脚の状態，脚先の座標，脚接地の基準点の座標，重心の位置，ロールピッチヨーの回転角度，親ノードの番号，次の動作，深さを持つ．
//! @n すなわちロボットの現在の状態を表している．しかし，脚の関節角度など一部の情報を持ってはいない．
//! @n これは実時間内に，グラフ探索を終えるための工夫である．
//! @n 先行研究のプログラムに比べて大きく内容を変更したが，基本的な構造は同じである．
struct SNode
{

	SNode();
	SNode(const SNode& other) = default;
	SNode(SNode&& other) noexcept = default;
	SNode& operator=(const SNode& other) = default;
	~SNode() = default;


	//! @brief 初期化関数
	//! @param [in] random 初期位置をランダムにするかどうか
	void Init(bool random);

	//! @brief 重心位置を変更する関数．
	//! @param [in] new_com 新しい重心位置
	//! @param [in] do_change_leg_base_pos 遊脚中の脚の接地基準地点の座標を変更するかどうか
	//! @details 脚位置は脚の付け根からの相対座標で表現されている． 
	//! @n 遊脚している脚は一緒に移動するが，接地脚は移動しないため座標を変更してやる必要がある．
	void ChangeGlobalCenterOfMass(const designlab::Vector3& new_com, const bool do_change_leg_base_pos);

	//! @brief 自身を親ノードに変更する関数．
	//! @details depthを0に，parent_numを-1に初期化する．
	constexpr void ChangeParentNode()
	{
		depth = 0;
		parent_num = -1;
	}

	//! @brief 次の動作を設定する関数．
	//! @param [in] parent 親ノードの番号
	//! @param [in] next 次の動作
	//! @details 深さを一つ深くして，親と次の動作を設定する．
	constexpr void ChangeToNextNode(const int parent, const EHexapodMove next)
	{
		++depth;
		parent_num = parent;
		next_move = next;
	}

	//! @brief ノードの情報を文字列に変換する関数．
	//! @n デバッグ用に詳細な情報を出力する．
	//! @return ノードの情報を文字列にしたもの
	std::string ToString() const;


	//! 比較演算子
	constexpr bool operator == (const SNode& other) const
	{
		if (next_move != other.next_move) { return false; }

		if (leg_state != other.leg_state) { return false; }

		for (int i = 0; i < HexapodConst::LEG_NUM; i++)
		{
			if (leg_pos[i] != other.leg_pos[i]) { return false; }
			//if (leg_base_pos[i] != other.leg_base_pos[i]) { return false; }
		}

		if (global_center_of_mass != other.global_center_of_mass) { return false; }
		if (rot != other.rot) { return false; }

		return true;
	}

	constexpr bool operator != (const SNode& other) { return !(*this == other); }



	dl_leg::LegStateBit leg_state;		//!< [4 byte] 脚状態，重心パターンをbitで表す．leg_state.hにこれを操作する用の名前空間と関数がある．旧名 leg_con

	std::array<::designlab::Vector3, HexapodConst::LEG_NUM> leg_pos;			//!< [4 * 3 * 6 = 72 byte] 脚先の座標．(coxa(脚の付け根)を原点とする)
	std::array<::designlab::Vector3, HexapodConst::LEG_NUM> leg_base_pos;		//!< [4 * 3 * 6 = 72 byte] 脚接地の基準点の座標．離散化した時に4になる地点．(coxa(脚の付け根)を原点とする)
	::designlab::Vector3 global_center_of_mass;		//!< [4 * 3 = 12byte] グローバル座標における重心の位置．旧名 GCOM
	::designlab::EulerXYZ rot;						//!< [4 * 3 = 12byte] ロール(X軸) ピッチ(Y軸) ヨー(Z軸)　右ねじの方向を正回転とし，ロボットの重心を原点に回転する．旧名 ThP ThR ThY

	EHexapodMove next_move;		//!< [4 byte] 次の動作を代入する．元のプログラムではint debugが担っていた仕事を行う．
	int parent_num;				//!< [4 byte] 自身の親がvector配列のどこにいるのかを記録する．親がいないなら負の値をとる．
	int depth;					//!< [4 byte] 自身の深さ．一番上の親が深さ0となる．
};


std::ostream& operator<<(std::ostream& stream, const SNode& value);

