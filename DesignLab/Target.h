//! @file Target.h
//! @brief ロボットの目標地点や目標姿勢を表現する構造体

#ifndef DESIGNLAB_TARGET_H_
#define DESIGNLAB_TARGET_H_

#include "designlab_quaternion.h"
#include "designlab_vector3.h"


//! @enum TargetMode
//! @brief Targetへの移動をどのように評価するか表す列挙体
//! @details [列挙体について]
//! @n 以下で定義されているTargetModeが列挙体に当たる
//! @n イメージとしては以下に宣言された値しかとることのできない新しい変数を定義するような感じ．
//! @n こうすることで関数を特定のモードで動作させたいときなどに，入力されたくない数字が入力されずらくなる．
//! @n また値に名前を付けることで何を想定してこの値が代入されたのかが分かりやすくなるという利点がある．
//! @n C++ には enum と enum class という2通りの列挙体があるが，安全のため enum class の使用をおすすめする．
//! @n よくわからんっていうのであれば TargetMode の記述を真似して書いてみて．
enum class TargetMode : int
{
	kNone,					//!< 評価しない．
	kStraightMoveVector,	//!< 直線移動をさせる．（移動したい方向をベクトルで示す）
	kStraightMovePosition,	//!< 直線移動をさせる．（移動したい座標を示す）
	kSpotTurnDirection,		//!< その場で旋回させる．（最終的な姿勢を示す）．
	kSpotTurnAngle,			//!< その場で旋回させる．（回転角度を示す）．
	kTurnDirection,			//!< 旋回動作．（方向）．
	kTurnAngle,				//!< 旋回動作（角度）．
};


//! @struct TargetRobotState
//! @brief 探索において目標となる座標や角度，評価する値についてまとめた構造体
struct TargetRobotState
{
	::designlab::Vector3 target_direction{ 0.f, 0.f, 0.f };	//!< 目標方向
	::designlab::Vector3 target_position{ 0.f, 0.f, 0.f };	//!< 目標位置
	::designlab::Quaternion target_quat{ 1.f, 0.f, 0.f, 0.f };	//!< 目標角度
	::designlab::Vector3 turn;		//!< 目標旋回方向（反時計回り→正）
	::designlab::Vector3 turning_center{ 0.f, 0.f, 0.f };		//!< 回転中心
	float turning_radius{ 0.f };				//!< 旋回半径

	TargetMode target_mode{ TargetMode::kNone };	//!< どうやって目標を評価するか
};


#endif	// DESIGNLAB_TARGET_H_