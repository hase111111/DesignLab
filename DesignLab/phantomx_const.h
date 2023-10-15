//! @file phantomx_const.h
//! @brief PhantomXのステータスをまとめた定数クラス．

#ifndef DESIGNLAB_PHANTOMX_CONST_H_
#define DESIGNLAB_PHANTOMX_CONST_H_

#include <array>

#include "cassert_define.h"
#include "designlab_math_util.h"


//! @class PhantomXConst
//! @brief PhantomXのパラメータを定数で表現したもの．
//! @n コンストラクタを削除したので，実体は生成できない．( PhantomXConst::kLegNum みたいに値を呼び出すこと )
//! @details 簡単のため値をここにまとめたが，むやみにこの値を参照せずにHexapodStateCalculatorを使うこと．
//! @n また，座標系はロボット前方にx軸，左方向にy軸，上方向にz軸をとる右手座標系である．
class PhantomXConst final
{
private:
	constexpr static  int kPhantomXLegNum = 6;

public:

	// コンストラクタとコピーコンストラクタを削除し，実体を生成できないようにする．
	PhantomXConst() = delete;
	PhantomXConst(const PhantomXConst& other) = delete;
	PhantomXConst& operator=(const PhantomXConst& other) = delete;
	PhantomXConst(PhantomXConst&& other) = delete;

	//! 第1関節の初期角度[rad]
	constexpr static std::array<float, kPhantomXLegNum> kCoxaDefaultAngle = {
		::designlab::math_util::ConvertDegToRad(-45.0f), 
		::designlab::math_util::ConvertDegToRad(-90.0f), 
		::designlab::math_util::ConvertDegToRad(-135.0f),
		::designlab::math_util::ConvertDegToRad(135.0f), 
		::designlab::math_util::ConvertDegToRad(90.0f),  
		::designlab::math_util::ConvertDegToRad(45.0f) 
	};

	constexpr static float kCoxaAngleMin = ::designlab::math_util::ConvertDegToRad(-81.0f);	//!< 第1関節の可動範囲の最小値[rad]．詳しくはreferenceをフォルダ参照．
	constexpr static float kCoxaAngleMax = ::designlab::math_util::ConvertDegToRad(81.0f);	//!< 第1関節の可動範囲の最大値[rad]．詳しくはreferenceをフォルダ参照．

	constexpr static float kFEMUR_ANGLE_MIN = ::designlab::math_util::ConvertDegToRad(-102.0f);	//!< 第2関節の可動範囲の最小値[rad]．詳しくはreferenceをフォルダ参照．
	constexpr static float kFEMUR_ANGLE_MAX = ::designlab::math_util::ConvertDegToRad(100.0f);	//!< 第2関節の可動範囲の最大値[rad]．詳しくはreferenceをフォルダ参照．

	constexpr static float kTIBIA_ANGLE_MIN = ::designlab::math_util::ConvertDegToRad(-135.0f);	//!< 第2関節の可動範囲の最小値[rad]．詳しくはreferenceをフォルダ参照．
	constexpr static float kTIBIA_ANGLE_MAX = ::designlab::math_util::ConvertDegToRad(40.0f);	//!< 第2関節の可動範囲の最大値[rad]．詳しくはreferenceをフォルダ参照．

	//!< 第2関節は曲がっているので，直線的に考えるためのオフセット角度[rad]．
	constexpr static float kFemurVirtualLinkOffsetAngle = ::designlab::math_util::ConvertDegToRad(-13.5f);	

	//!< 第3関節は曲がっているので，直線的に考えるためのオフセット角度[rad]．
	constexpr static float kTibiaVirtualLinkOffsetAangle = ::designlab::math_util::ConvertDegToRad(13.5f);


	constexpr static float kCoxaLength = 52.0f;		//!< 第1関節部の長さ[mm]．詳しくはreferenceをフォルダ参照．
	constexpr static float kFemurLength = 66.0f;	//!< 第2関節部の長さ[mm]．詳しくはreferenceをフォルダ参照．
	constexpr static float kTibiaLength = 130.0f;	//!< 第3関節部の長さ[mm]．詳しくはreferenceをフォルダ参照．(163mm)

	constexpr static float kCoxaBaseOffsetY = 61.64f;		//!< coxa linkの付け根(前方・後方)までの長さ[mm]．
	constexpr static float kCenterCoxaBaseOffsetY = 103.4f;	//!< coxa linkの付け根(中央)までの長さ[mm]．
	constexpr static float kCoxaBaseOffsetX = 124.8f;		//!< coxa linkの付け根(前方・後方)までの長さ[mm]．
	constexpr static float BODY_HEIGHT = 40.0f;			//!< 胴体の高さ[mm]．


	//! @brief 第1関節の角度が有効な範囲内かどうかを判定する．
	//! @param [in] leg_index 脚の番号．
	//! @param [in] angle 判定する角度．
	//! @return bool 有効な範囲内ならtrue．
	constexpr static bool IsVaildCoxaAngle(const int leg_index, const float angle)
	{
		assert(0 <= leg_index && leg_index < kPhantomXLegNum);

		return (
			kCoxaAngleMin + kCoxaDefaultAngle[leg_index] <= angle && 
			angle <= kCoxaAngleMax + kCoxaDefaultAngle[leg_index]
		);
	};

	//! @biref 第2関節の角度が有効な範囲内かどうかを判定する．
	//! @param [in] angle 判定する角度．
	//! @return bool 有効な範囲内ならtrue．
	constexpr static bool IsVaildFemurAngle(const float angle)
	{
		return (kFEMUR_ANGLE_MIN <= angle && angle <= kFEMUR_ANGLE_MAX);
	};

	//! @brief 第3関節の角度が有効な範囲内かどうかを判定する．
	//! @param [in] angle 判定する角度．
	//! @return bool 有効な範囲内ならtrue．
	constexpr static bool IsVaildTibiaAngle(const float angle)
	{
		return (kTIBIA_ANGLE_MIN <= angle && angle <= kTIBIA_ANGLE_MAX);
	};
};

#endif 