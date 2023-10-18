//! @file phantomx_const.h
//! @brief PhantomX�̃X�e�[�^�X���܂Ƃ߂��萔�N���X�D

#ifndef DESIGNLAB_PHANTOMX_CONST_H_
#define DESIGNLAB_PHANTOMX_CONST_H_

#include <array>

#include "cassert_define.h"
#include "designlab_math_util.h"


//! @class PhantomXConst
//! @brief PhantomX�̃p�����[�^��萔�ŕ\���������́D
//! @n �R���X�g���N�^���폜�����̂ŁC���̂͐����ł��Ȃ��D( PhantomXConst::kLegNum �݂����ɒl���Ăяo������ )
//! @details �ȒP�̂��ߒl�������ɂ܂Ƃ߂����C�ނ�݂ɂ��̒l���Q�Ƃ�����HexapodStateCalculator���g�����ƁD
//! @n �܂��C���W�n�̓��{�b�g�O����x���C��������y���C�������z�����Ƃ�E����W�n�ł���D
class PhantomXConst final
{
private:
	constexpr static  int kPhantomXLegNum = 6;

public:

	// �R���X�g���N�^�ƃR�s�[�R���X�g���N�^���폜���C���̂𐶐��ł��Ȃ��悤�ɂ���D
	PhantomXConst() = delete;
	PhantomXConst(const PhantomXConst& other) = delete;
	PhantomXConst& operator=(const PhantomXConst& other) = delete;
	PhantomXConst(PhantomXConst&& other) = delete;

	//! ��1�֐߂̏����p�x[rad]
	constexpr static std::array<float, kPhantomXLegNum> kCoxaDefaultAngle = {
		::designlab::math_util::ConvertDegToRad(-45.0f), 
		::designlab::math_util::ConvertDegToRad(-90.0f), 
		::designlab::math_util::ConvertDegToRad(-135.0f),
		::designlab::math_util::ConvertDegToRad(135.0f), 
		::designlab::math_util::ConvertDegToRad(90.0f),  
		::designlab::math_util::ConvertDegToRad(45.0f) 
	};

	constexpr static float kCoxaAngleMin = ::designlab::math_util::ConvertDegToRad(-81.0f);	//!< ��1�֐߂̉��͈͂̍ŏ��l[rad]�D�ڂ�����reference���t�H���_�Q�ƁD
	constexpr static float kCoxaAngleMax = ::designlab::math_util::ConvertDegToRad(81.0f);	//!< ��1�֐߂̉��͈͂̍ő�l[rad]�D�ڂ�����reference���t�H���_�Q�ƁD

	constexpr static float kFEMUR_ANGLE_MIN = ::designlab::math_util::ConvertDegToRad(-105.0f);	//!< ��2�֐߂̉��͈͂̍ŏ��l[rad]�D�ڂ�����reference���t�H���_�Q�ƁD
	constexpr static float kFEMUR_ANGLE_MAX = ::designlab::math_util::ConvertDegToRad(100.0f);	//!< ��2�֐߂̉��͈͂̍ő�l[rad]�D�ڂ�����reference���t�H���_�Q�ƁD

	constexpr static float kTIBIA_ANGLE_MIN = ::designlab::math_util::ConvertDegToRad(-115.0f);	//!< ��2�֐߂̉��͈͂̍ŏ��l[rad]�D�ڂ�����reference���t�H���_�Q�ƁD
	constexpr static float kTIBIA_ANGLE_MAX = ::designlab::math_util::ConvertDegToRad(56.0f);	//!< ��2�֐߂̉��͈͂̍ő�l[rad]�D�ڂ�����reference���t�H���_�Q�ƁD

	//!< ��2�֐߂͋Ȃ����Ă���̂ŁC�����I�ɍl���邽�߂̃I�t�Z�b�g�p�x[rad]�D
	constexpr static float kFemurVirtualLinkOffsetAngle = ::designlab::math_util::ConvertDegToRad(-13.5f);	

	//!< ��3�֐߂͋Ȃ����Ă���̂ŁC�����I�ɍl���邽�߂̃I�t�Z�b�g�p�x[rad]�D
	constexpr static float kTibiaVirtualLinkOffsetAangle = ::designlab::math_util::ConvertDegToRad(13.5f);


	constexpr static float kCoxaLength = 52.0f;		//!< ��1�֐ߕ��̒���[mm]�D�ڂ�����reference���t�H���_�Q�ƁD
	constexpr static float kFemurLength = 66.0f;	//!< ��2�֐ߕ��̒���[mm]�D�ڂ�����reference���t�H���_�Q�ƁD(���m�ȃX�e�[�^�X��66.061mm)
	constexpr static float kTibiaLength = 137.0f;	//!< ��3�֐ߕ��̒���[mm]�D�ڂ�����reference���t�H���_�Q�ƁD(���X�e�[�^�X130mm)

	constexpr static float kCoxaBaseOffsetY = 61.64f;		//!< coxa link�̕t����(�O���E���)�܂ł̒���[mm]�D
	constexpr static float kCenterCoxaBaseOffsetY = 103.4f;	//!< coxa link�̕t����(����)�܂ł̒���[mm]�D
	constexpr static float kCoxaBaseOffsetX = 124.8f;		//!< coxa link�̕t����(�O���E���)�܂ł̒���[mm]�D
	constexpr static float kCoxaBaseOffsetZ = 1.116f;		//!< coxa link�̕t�����܂ł̒���(�����)[mm]�D
	constexpr static float BODY_HEIGHT = 40.0f;			//!< ���̂̍���[mm]�D


	//! @brief ��1�֐߂̊p�x���L���Ȕ͈͓����ǂ����𔻒肷��D
	//! @param [in] leg_index �r�̔ԍ��D
	//! @param [in] angle ���肷��p�x�D
	//! @return bool �L���Ȕ͈͓��Ȃ�true�D
	constexpr static bool IsVaildCoxaAngle(const int leg_index, const float angle)
	{
		assert(0 <= leg_index && leg_index < kPhantomXLegNum);

		return (
			kCoxaAngleMin + kCoxaDefaultAngle[leg_index] <= angle && 
			angle <= kCoxaAngleMax + kCoxaDefaultAngle[leg_index]
		);
	};

	//! @biref ��2�֐߂̊p�x���L���Ȕ͈͓����ǂ����𔻒肷��D
	//! @param [in] angle ���肷��p�x�D
	//! @return bool �L���Ȕ͈͓��Ȃ�true�D
	constexpr static bool IsVaildFemurAngle(const float angle)
	{
		return (kFEMUR_ANGLE_MIN <= angle && angle <= kFEMUR_ANGLE_MAX);
	};

	//! @brief ��3�֐߂̊p�x���L���Ȕ͈͓����ǂ����𔻒肷��D
	//! @param [in] angle ���肷��p�x�D
	//! @return bool �L���Ȕ͈͓��Ȃ�true�D
	constexpr static bool IsVaildTibiaAngle(const float angle)
	{
		return (kTIBIA_ANGLE_MIN <= angle && angle <= kTIBIA_ANGLE_MAX);
	};
};

#endif 