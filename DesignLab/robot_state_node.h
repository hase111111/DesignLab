//! @file robot_state_node.h
//! @brief 歩容パターングラフのノード(頂点)を表す構造体．
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


#ifndef DESIGNLAB_ROBOT_STATE_NODE_H_
#define DESIGNLAB_ROBOT_STATE_NODE_H_

#include <array>
#include <iostream>
#include <memory>
#include <string>

#include <magic_enum.hpp>

#include "array_util.h"
#include "hexapod_next_move.h"
#include "interface_hexapod_coordinate_converter.h"
#include "leg_state.h"
#include "math_quaternion.h"
#include "math_vector3.h"
#include "robot_operation.h"


namespace designlab
{

//! @struct RobotStateNode
//! @brief グラフ構造のためのノード(頂点)．旧名 LNODE
//! @details この構造体は，グラフ構造のためのノード(頂点)である． 
//! @n ノードは，脚の状態，脚先の座標，脚接地の基準点の座標，重心の位置，ロールピッチヨーの回転角度，親ノードの番号，次の動作，深さを持つ．
//! @n すなわちロボットの現在の状態を表している．しかし，脚の関節角度など一部の情報を持ってはいない．
//! @n これは実時間内に，グラフ探索を終えるための工夫である．
//! @n 先行研究のプログラムに比べて大きく内容を変更したが，基本的な構造は同じである．
struct RobotStateNode final
{
	static constexpr int kNoParentIndex = -1;	//!< 親がいないことを表す値．

	constexpr RobotStateNode() :
		leg_state(0),
		leg_pos{ MakeArray<Vector3>(
			Vector3{},
			Vector3{},
			Vector3{},
			Vector3{},
			Vector3{},
			Vector3{}
		)
		},
		leg_reference_pos{ leg_pos },
		global_center_of_mass{},
		quat{},
		next_move(enums::HexapodMove::kComUpDown),
		parent_index(kNoParentIndex),
		depth(0)
	{
	}

	constexpr RobotStateNode(
		const leg_func::LegStateBit& state,
		const std::array<Vector3, HexapodConst::kLegNum>& pos,
		const std::array<Vector3, HexapodConst::kLegNum>& ref_pos,
		const Vector3& com,
		const Quaternion& q,
		const enums::HexapodMove move,
		const int parent,
		const int d
	) :
		leg_state(state),
		leg_pos(pos),
		leg_reference_pos(ref_pos),
		global_center_of_mass(com),
		quat(q),
		next_move(move),
		parent_index(parent),
		depth(d)
	{
	}

	RobotStateNode(const RobotStateNode& other) = default;
	RobotStateNode(RobotStateNode&& other) noexcept = default;
	RobotStateNode& operator=(const RobotStateNode& other) = default;
	~RobotStateNode() = default;

	//! @brief 重心位置を変更する関数．
	//! @param [in] new_com 新しい重心位置．グローバル座標系．
	//! @param [in] do_change_leg_base_pos 遊脚中の脚の接地基準地点の座標を変更するかどうか
	//! @details 脚位置は脚の付け根からの相対座標で表現されている． 
	//! @n 遊脚している脚は一緒に移動するが，接地脚は移動しないため座標を変更してやる必要がある．
	void ChangeGlobalCenterOfMass(const designlab::Vector3& new_com, bool do_change_leg_base_pos);

	//! @brief クォータニオンを変更し，胴体を回転させる関数．
	//! @param [in] new_quat 新しいクォータニオン．
	void ChangeQuat(const std::shared_ptr<const IHexapodCoordinateConverter>& converter_ptr, const designlab::Quaternion& new_quat);

	//! @brief 自身を根ノードに変更する関数．
	//! @n depthを0に，parent_numを-1に初期化する．
	inline void ChangeLootNode()
	{
		depth = 0;
		parent_index = kNoParentIndex;
		quat.Normalize();
	}

	//! @brief 次の動作を設定する関数．
	//! @n 深さを一つ深くして，親と次の動作を設定する．
	//! @param [in] parent_index_ 親ノードの番号.
	//! @param [in] next_move_ 次の動作．
	constexpr void ChangeToNextNode(const int parent_index_, const enums::HexapodMove next_move_)
	{
		++depth;
		parent_index = parent_index_;
		next_move = next_move_;
	}

	//! @brief 自身が根ノードであるか判定する．
	//! @return 自身が根ノードならtrue，そうでなければfalse．
	constexpr bool IsLootNode() const
	{
		return (parent_index < 0) && (depth == 0);
	}

	//! @brief ノードの情報を文字列に変換する関数．
	//! @n デバッグ用に詳細な情報を出力する．
	//! @return ノードの情報を文字列にしたもの．
	std::string ToString() const;

	//! @brief ノードの情報をcsv形式の文字列に変換する関数．
	//! @n カンマ区切りで出力する．
	//! @return ノードの情報をcsv形式の文字列にしたもの．
	std::string ToCsvString() const;

	//! @brief 文字列をノードの情報に変換する関数．
	//! @param [in] str ノードの情報をまとめた文字列，カンマ区切り．
	//! @return  RobotStateNode ロボットの状態を表すノード．
	static RobotStateNode FromString(const std::string& str);


	//! 比較演算子
	constexpr bool operator == (const RobotStateNode& other) const
	{
		if (next_move != other.next_move) { return false; }

		if (leg_state != other.leg_state) { return false; }

		for (int i = 0; i < HexapodConst::kLegNum; i++)
		{
			if (leg_pos[i] != other.leg_pos[i]) { return false; }
			//if (leg_reference_pos[i] != other.leg_reference_pos[i]) { return false; }
		}

		if (global_center_of_mass != other.global_center_of_mass) { return false; }
		if (quat != other.quat) { return false; }

		return true;
	}

	constexpr bool operator != (const RobotStateNode& other) { return !(*this == other); }



	leg_func::LegStateBit leg_state;	//!< [4 byte] 脚状態，重心パターンをbitで表す．leg_state.hにこれを操作する用の名前空間と関数がある．旧名 leg_con

	std::array<Vector3, HexapodConst::kLegNum> leg_pos;			//!< [4 * 3 * 6 = 72 byte] 脚先の座標．(coxa(脚の付け根)を原点とする)
	std::array<Vector3, HexapodConst::kLegNum> leg_reference_pos;	//!< [4 * 3 * 6 = 72 byte] 脚接地の基準点の座標．離散化した時に4になる地点．(coxa(脚の付け根)を原点とする)
	Vector3 global_center_of_mass;	//!< [4 * 3 = 12byte] グローバル座標における重心の位置．旧名 GCOM
	Quaternion quat;				//!< [4 * 4 = 16byte] 姿勢を表すクォータニオン．

	enums::HexapodMove next_move;		//!< [4 byte] 次の動作を代入する．元のプログラムではint debugが担っていた仕事を行う．
	int parent_index;			//!< [4 byte] 自身の親がvector配列のどこにいるのかを記録する．親がいないなら負の値をとる．
	int depth;					//!< [4 byte] 自身の深さ．一番上の親が深さ0となる．
};


template <class Char>
std::basic_ostream<Char>& operator <<(std::basic_ostream<Char>& os, const RobotStateNode& value)
{
	os << value.leg_state.to_string() << ",";

	for (int i = 0; i < HexapodConst::kLegNum; i++)
	{
		os << value.leg_pos[i] << ",";
	}

	for (int i = 0; i < HexapodConst::kLegNum; i++)
	{
		os << value.leg_reference_pos[i] << ",";
	}

	os << value.global_center_of_mass << ",";
	os << value.quat << ",";
	os << magic_enum::enum_name(value.next_move) << ",";
	os << value.parent_index << ",";
	os << value.depth;

	return os;
}

}	// namespace designlab


#endif	// DESIGNLAB_ROBOT_STATE_NODE_H_