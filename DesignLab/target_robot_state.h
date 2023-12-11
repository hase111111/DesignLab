//! @file target_robot_state.h
//! @brief ロボットの目標地点や目標姿勢を表現する構造体

#ifndef DESIGNLAB_TARGET_ROBOT_STATE_H_
#define DESIGNLAB_TARGET_ROBOT_STATE_H_

#include "designlab_quaternion.h"
#include "designlab_vector3.h"


namespace designlab
{
	//! @namespace designlab::enums
	//! @brief 列挙体をまとめた名前空間．
	namespace enums
	{
		//! @enum TargetMode
		//! @brief Targetへの移動をどのように評価するか表す列挙体．
		//! @details [列挙体について]
		//! @n 以下で定義されているTargetModeが列挙体に当たる．
		//! @n イメージとしては以下に宣言された値しかとることのできない新しい変数を定義するような感じ．
		//! @n こうすることで関数を特定のモードで動作させたいときなどに，入力されたくない数字が入力されなくなる．
		//! @n また値に名前を付けることで何を想定してこの値が代入されたのかが分かりやすくなるという利点がある．
		//! @n C++ には enum と enum class という2通りの列挙体があるが，安全のため enum class の使用をおすすめする．
		//! @n よくわからない場合は， TargetMode の記述を真似してみること．
		enum class TargetMode : int
		{
			kNone,					//!< 評価しない．
			kStraightMoveVector,	//!< 直線移動をさせる（移動したい方向をベクトルで示す）．
			kStraightMovePosition,	//!< 直線移動をさせる（移動したい座標を示す）．
			kSpotTurnLastPosture,	//!< その場で旋回させる（最終的な姿勢 Posture を示す）．
			kSpotTurnRotAxis,		//!< その場で旋回させる（回転軸を示し，その軸周りの右ねじの回転）．
		};
	}
}


//! @struct TargetRobotState
//! @brief 探索において目標となる座標や角度，評価する値についてまとめた構造体
struct TargetRobotState final
{
	::designlab::Vector3 straight_move_vector_;		//!< 目標方向．正規化されたベクトル．
	::designlab::Vector3 straight_move_position_;	//!< 目標位置（グローバル座標）．
	::designlab::Quaternion spot_turn_last_posture_;	//!< 目標姿勢(posture)．
	::designlab::Vector3 spot_turn_rot_axis;			//!< 旋回時の回転軸．右ねじの回転．

	::designlab::enums::TargetMode target_mode{ ::designlab::enums::TargetMode::kStraightMovePosition };	//!< どうやって目標を評価するか
};


#endif	// DESIGNLAB_TARGET_ROBOT_STATE_H_
