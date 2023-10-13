//! @file phantomx_const.h
//! @brief PhantomX�̃X�e�[�^�X���܂Ƃ߂��萔�N���X�D

#ifndef DESIGNLAB_PHANTOMX_CONST_H_
#define DESIGNLAB_PHANTOMX_CONST_H_

#include <array>

#include "designlab_math_util.h"


class PhantomXConst final
{
public:

	PhantomXConst() = delete;
	PhantomXConst(const PhantomXConst& other) = delete;
	PhantomXConst& operator=(const PhantomXConst& other) = delete;
	PhantomXConst(PhantomXConst&& other) = delete;

	//! ��1�֐߂̏����p�x[rad]
	constexpr static std::array<float, 6> kCoxaDefaultAngle = {
		::designlab::math_util::ConvertDegToRad(-45.0f), ::designlab::math_util::ConvertDegToRad(-90.0f), ::designlab::math_util::ConvertDegToRad(-135.0f),
		::designlab::math_util::ConvertDegToRad(135.0f), ::designlab::math_util::ConvertDegToRad(90.0f),  ::designlab::math_util::ConvertDegToRad(45.0f) 
	};
};

#endif 