//! @file Target.h
//! @brief ロボットの目標地点や目標姿勢を表現する構造体


#ifndef DESIGNLAB_TARGET_H_
#define DESIGNLAB_TARGET_H_

#include "designlab_vector3.h"
#include "designlab_euler.h"


//! @todo : 先行研究のものなので残しているが，手法を丸っと書きなおすべきだろう．

//! @enum TargetMode
//! @brief Targetへの移動をどのように評価するか表す列挙体
//! @details [列挙体について]
//! @n 以下で定義されているETargetModeが列挙体に当たる
//! @n イメージとしては以下に宣言された値しかとることのできない新しい変数を定義するような感じ．
//! @n こうすることで関数を特定のモードで動作させたいときなどに，入力されたくない数字が入力されずらくなる．
//! @n また値に名前を付けることで何を想定してこの値が代入されたのかが分かりやすくなるという利点がある．
//! @n C++ には enum と enum class という2通りの列挙体があるが，安全のため enum class の使用をおすすめする．
//! @n よくわからんっていうのであれば TargetMode の記述を真似して書いてみて．
enum class TargetMode : int
{
	kNone,				//!< 評価しない
	kStraightVector,		//!< 直線移動（移動したい方向をベクトルで示す）
	kStraightPosition,	//!< 直線移動（移動したい座標を示す）
	kTurnSpotDirection,	//!< その場旋回（最終的に向くべき回転方向を示す）
	kTurnSpotAngle,		//!< その場旋回（回転角度を示す）
	kTurnDirection,		//!< 旋回（方向）
	kTurnAngle,			//!< 旋回（角度）
};


//! @struct TargetRobotState
//! @brief 探索において目標となる座標や角度，評価する値についてまとめた構造体
struct TargetRobotState
{
	TargetRobotState() : 
		target_direction({ 0,0,0 }), 
		target_position({ 0,0,0 }), 
		target_rotation({ 0,0,0 }),
		target_angle({ 0,0,0 }), 
		rotation_center({ 0,0,0 }), 
		turning_radius(0), 
		target_mode(TargetMode::kNone)
	{
	}

	designlab::Vector3 target_direction;	//!< 目標方向
	designlab::Vector3 target_position;		//!< 目標位置
	designlab::EulerXYZ target_rotation;	//!< 目標角度
	designlab::EulerXYZ target_angle;		//!< 目標旋回方向（反時計回り→正）
	designlab::Vector3 rotation_center;		//!< 回転中心
	float turning_radius;				//!< 旋回半径
	TargetMode target_mode;				//!< どうやって目標を評価するか
};


#endif	// DESIGNLAB_TARGET_H_