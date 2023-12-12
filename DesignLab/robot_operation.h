//! @file robot_operation.h
//! @brief ロボットの目標地点や目標姿勢によって，ロボットの操作方法を表す構造体．

#ifndef DESIGNLAB_ROBOT_OPERATION_H_
#define DESIGNLAB_ROBOT_OPERATION_H_

#include "designlab_quaternion.h"
#include "designlab_vector3.h"
#include "toml_serialize_macro.h"


namespace designlab
{
	//! @namespace designlab::enums
	//! @brief 列挙体をまとめた名前空間．
	namespace enums
	{
		//! @enum RobotOperationType
		//! @brief Robotをどのように動かすかを表す列挙体．
		//! @details [列挙体について]
		//! @n 以下で定義されている RobotOperationType が列挙体に当たる．
		//! @n イメージとしては以下に宣言された値しかとることのできない新しい変数を定義するような感じ．
		//! @n こうすることで関数を特定のモードで動作させたいときなどに，入力されたくない数字が入力されなくなる．
		//! @n また値に名前を付けることで何を想定してこの値が代入されたのかが分かりやすくなるという利点がある．
		//! @n C++ には enum と enum class という2通りの列挙体があるが，安全のため enum class の使用をおすすめする．
		//! @n よくわからない場合は， RobotOperationType の記述を真似してみること．
		enum class RobotOperationType : int
		{
			kNone,					//!< 無効値．
			kStraightMoveVector,	//!< 直線移動をさせる（移動したい方向をベクトルで示す）．
			kStraightMovePosition,	//!< 直線移動をさせる（移動したい座標を示す）．
			kSpotTurnLastPosture,	//!< その場で旋回させる（最終的な姿勢 Posture を示す）．
			kSpotTurnRotAxis,		//!< その場で旋回させる（回転軸を示し，その軸周りの右ねじの回転）．
		};
	}
}


//! @struct RobotOperation
//! @brief 探索において目標となる座標や角度，評価する値についてまとめた構造体．
//! @details 
//! 先行研究では target という名前だった．
struct RobotOperation final
{
	::designlab::Vector3 straight_move_vector_{ 1.f, 0.f, 0.f };		//!< 目標方向．正規化されたベクトル．
	::designlab::Vector3 straight_move_position_{ 10000.f, 0.f, 0.f };	//!< 目標位置（グローバル座標）．
	::designlab::Quaternion spot_turn_last_posture_{ ::designlab::Quaternion::MakeByAngleAxis(0, ::designlab::Vector3::GetUpVec()) };	//!< 目標姿勢(posture)．
	::designlab::Vector3 spot_turn_rot_axis{ ::designlab::Vector3::GetUpVec() };			//!< 旋回時の回転軸．右ねじの回転．

	::designlab::enums::RobotOperationType operation_type{ ::designlab::enums::RobotOperationType::kStraightMovePosition };	//!< どうやって目標を評価するか
};


DESIGNLAB_TOML11_DESCRIPTION_CLASS(RobotOperation)
{
	DESIGNLAB_TOML11_FILE_NO_DESCRIPTION();

	DESIGNLAB_TOML11_TABLE_NO_DESCRIPTION();

	DESIGNLAB_TOML11_VARIABLE_ADD_DESCRIPTION(straight_move_vector_, "a", "float");
	DESIGNLAB_TOML11_VARIABLE_ADD_DESCRIPTION(straight_move_position_, "b", "float");
	DESIGNLAB_TOML11_VARIABLE_ADD_DESCRIPTION(spot_turn_last_posture_, "c", "float");
	DESIGNLAB_TOML11_VARIABLE_ADD_DESCRIPTION(spot_turn_rot_axis, "d", "float");

	DESIGNLAB_TOML11_VARIABLE_ADD_DESCRIPTION(operation_type, "e", "float");
};


DESIGNLAB_TOML11_SERIALIZE(RobotOperation, straight_move_vector_, straight_move_position_, spot_turn_last_posture_, spot_turn_rot_axis, operation_type);


#endif	// DESIGNLAB_ROBOT_OPERATION_H_