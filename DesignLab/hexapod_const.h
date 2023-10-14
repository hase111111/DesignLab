//! @file hexapod_const.h
//! @brief Hexapodの定数をまとめたクラス．


#ifndef DESIGNLAB_HEXAPOD_CONST_H_
#define DESIGNLAB_HEXAPOD_CONST_H_


#include "designlab_math_util.h"


//! @class HexapodConst
//! @brief Hexapod，phantomXのパラメータを定数で表現したもの．他の6脚ロボットのパラメータもここに追加する(今のところ予定はないが)．
//! @details コンストラクタを削除したので，実体は生成できない．( HexapodConst::kLegNum みたいに値を呼び出すこと )
class HexapodConst final
{
public:

	//コンストラクタとコピーコンストラクタを削除し，実体を生成できないようにする．
	HexapodConst() = delete;
	HexapodConst(const HexapodConst& other) = delete;
	HexapodConst& operator=(const HexapodConst& other) = delete;
	HexapodConst(HexapodConst&& other) = delete;


	constexpr static int kLegNum = 6;	//!< Hexapodの脚の本数を表す．これを変更しても脚の本数が変更できるわけではない．マジックナンバーをなくすことが目的．

	// PhantomXのパラメータ

	//ロボットの可動範囲
	const static float VERTICAL_MAX_RANGE;			//!< 地面の最高点と胴体下方の隙間の最大値を示す．脚を伸ばし切らない程度に設定する．旧名 MAX_DELTAZ
	const static float VERTICAL_MIN_RANGE;			//!< 地面の最高点と胴体下方の隙間の最小値を示す．旧名 MIN_DELTAZ

	constexpr static float MOVABLE_LEG_RANGE = ::designlab::math_util::ConvertDegToRad(40.0f);			//!< coxa関節(第1関節)の稼働可能な角度 [rad]

	//! MOVABLE_LEG_RANGEのcos値 -85°，-130°，-175°，95°，50°，5°
	constexpr static float MOVABLE_LEG_RANGE_COS_MAX[kLegNum] = { 0.08715574f , -0.6427876f, -0.9961947f, -0.08715574f, 0.6427876f, 0.9961947f };

	//! MOVABLE_LEG_RANGEのcos値 -5°，-50°，-95°，175°，130°，85°
	constexpr static float MOVABLE_LEG_RANGE_COS_MIN[kLegNum] = { 0.9961947f,  0.6427876f, -0.08715574f, -0.9961947f, -0.6427876f, 0.08715574f };

	//! MOVABLE_LEG_RANGEのsin値 -5°，-50°，-95°，175°，130°，85°
	constexpr static float MOVABLE_LEG_RANGE_SIN_MAX[kLegNum] = { -0.08715574f, -0.76604444f, -0.9961947f, 0.0871557f, 0.76604444f, 0.9961946f };

	//! MOVABLE_LEG_RANGEのsin値 -85°，-130°，-175°，95°，50°，5°
	constexpr static float MOVABLE_LEG_RANGE_SIN_MIN[kLegNum] = { -0.9961947f, -0.76604444f, -0.08715574f, 0.9961947f, 0.76604444f,0.08715574f };


	//! 脚の第1関節の初期角度を示す[rad]．ロボットの正面を 0[rad]として，右ねじを正にとる．
	constexpr static float DEFAULT_LEG_ANGLE[kLegNum] = { 
		::designlab::math_util::ConvertDegToRad(-45.0f),	::designlab::math_util::ConvertDegToRad(-90.0f),::designlab::math_util::ConvertDegToRad(-135.0f),
		::designlab::math_util::ConvertDegToRad(135.0f),	::designlab::math_util::ConvertDegToRad(90.0f),	::designlab::math_util::ConvertDegToRad(45.0f) 
	};

	//! DEFAULT_LEG_ANGLEの値を元にsin cos を計算しておく．
	constexpr static float DEFAULT_LEG_ANGLE_SIN[kLegNum] = { -0.70710678118654f, -1.0f, -0.70710678118654f,
													 0.70710678118654f, 1.0f, 0.70710678118654f };

	constexpr static float DEFAULT_LEG_ANGLE_COS[kLegNum] = { 0.70710678118654f, 0.0f, -0.70710678118654f,
														 -0.70710678118654f, 0.0f, 0.70710678118654f };


};


#endif // !DESIGNLAB_HEXAPOD_CONST_H_