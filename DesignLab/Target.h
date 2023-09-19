#pragma once

#include "designlab_vector3.h"
#include "designlab_rotator.h"


enum class ETargetMode : int
{
	None,				//!< 評価しない
	StraightVector,		//!< 直線移動（移動したい方向をベクトルで示す）
	StraightPosition,	//!< 直線移動（移動したい座標を示す）
	TurnSpotDirection,	//!< その場旋回（最終的に向くべき回転方向を示す）
	TurnSpotAngle,		//!< その場旋回（回転角度を示す）
	TurnDirection,		//!< 旋回（方向）
	TurnAngle,			//!< 旋回（角度）
};


struct STarget
{
	STarget() : TargetDirection({ 0,0,0 }), TargetPosition({ 0,0,0 }), TargetRotation({ 0,0,0 }),
		TargetAngle({ 0,0,0 }), RotationCenter({ 0,0,0 }), TurningRadius(0), TargetMode(ETargetMode::None)
	{}

	designlab::Vector3 TargetDirection;	//!< 目標方向
	designlab::Vector3 TargetPosition;		//!< 目標位置
	designlab::SRotator TargetRotation;	//!< 目標角度
	designlab::SRotator TargetAngle;		//!< 目標旋回方向（反時計回り→正）
	designlab::Vector3 RotationCenter;		//!< 回転中心
	float TurningRadius;				//!< 旋回半径
	ETargetMode TargetMode;				//!< どうやって目標を評価するか
};

//! @file Target.h
//! @brief ロボットの目標地点や目標姿勢を表現する列挙体と構造体の実装．
//! @date 2023/07/23
//! @author 長谷川

//! @enum ETargetMode
//! @brief Targetへの移動をどのように評価するか表す列挙子
//! @details [列挙体について]<br>
//!	以下で定義されているETargetModeが列挙体に当たる<br>
//!	イメージとしては以下に宣言された値しかとることのできない新しい変数を定義するような感じ．<br>
//!	こうすることで関数を特定のモードで動作させたいときなどに，入力されたくない数字が入力されずらくなる．<br>
//!	また値に名前を付けることで何を想定してこの値が代入されたのかが分かりやすくなるという利点がある．<br>
//!	C++ には enum と enum class という2通りの列挙体があるが，安全のため enum class の使用をおすすめする．<br>
//!	よくわからんっていうのであれば ETargetMode の記述を真似して書いてみて．
//! @date 2023/07/23
//! @author 長谷川

//! @struct STarget
//! @brief 探索において目標となる座標や角度，評価する値についてまとめた構造体
//! @date 2023/07/23
//! @author 長谷川