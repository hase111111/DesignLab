//! @file hexapod_const.h
//! @brief Hexapod�̒萔���܂Ƃ߂��N���X�D


#ifndef DESIGNLAB_HEXAPOD_CONST_H_
#define DESIGNLAB_HEXAPOD_CONST_H_


#include "designlab_math_util.h"


//! @class HexapodConst
//! @brief Hexapod�CphantomX�̃p�����[�^��萔�ŕ\���������́D����6�r���{�b�g�̃p�����[�^�������ɒǉ�����(���̂Ƃ���\��͂Ȃ���)�D
//! @details �R���X�g���N�^���폜�����̂ŁC���̂͐����ł��Ȃ��D( HexapodConst::kLegNum �݂����ɒl���Ăяo������ )
class HexapodConst final
{
public:

	//�R���X�g���N�^�ƃR�s�[�R���X�g���N�^���폜���C���̂𐶐��ł��Ȃ��悤�ɂ���D
	HexapodConst() = delete;
	HexapodConst(const HexapodConst& other) = delete;
	HexapodConst& operator=(const HexapodConst& other) = delete;
	HexapodConst(HexapodConst&& other) = delete;


	constexpr static int kLegNum = 6;	//!< Hexapod�̋r�̖{����\���D�����ύX���Ă��r�̖{�����ύX�ł���킯�ł͂Ȃ��D�}�W�b�N�i���o�[���Ȃ������Ƃ��ړI�D

	// PhantomX�̃p�����[�^

	//���{�b�g�̉��͈�
	const static float VERTICAL_MAX_RANGE;			//!< �n�ʂ̍ō��_�Ɠ��̉����̌��Ԃ̍ő�l�������D�r��L�΂��؂�Ȃ����x�ɐݒ肷��D���� MAX_DELTAZ
	const static float VERTICAL_MIN_RANGE;			//!< �n�ʂ̍ō��_�Ɠ��̉����̌��Ԃ̍ŏ��l�������D���� MIN_DELTAZ

	constexpr static float MOVABLE_LEG_RANGE = ::designlab::math_util::ConvertDegToRad(40.0f);			//!< coxa�֐�(��1�֐�)�̉ғ��\�Ȋp�x [rad]

	//! MOVABLE_LEG_RANGE��cos�l -85���C-130���C-175���C95���C50���C5��
	constexpr static float MOVABLE_LEG_RANGE_COS_MAX[kLegNum] = { 0.08715574f , -0.6427876f, -0.9961947f, -0.08715574f, 0.6427876f, 0.9961947f };

	//! MOVABLE_LEG_RANGE��cos�l -5���C-50���C-95���C175���C130���C85��
	constexpr static float MOVABLE_LEG_RANGE_COS_MIN[kLegNum] = { 0.9961947f,  0.6427876f, -0.08715574f, -0.9961947f, -0.6427876f, 0.08715574f };

	//! MOVABLE_LEG_RANGE��sin�l -5���C-50���C-95���C175���C130���C85��
	constexpr static float MOVABLE_LEG_RANGE_SIN_MAX[kLegNum] = { -0.08715574f, -0.76604444f, -0.9961947f, 0.0871557f, 0.76604444f, 0.9961946f };

	//! MOVABLE_LEG_RANGE��sin�l -85���C-130���C-175���C95���C50���C5��
	constexpr static float MOVABLE_LEG_RANGE_SIN_MIN[kLegNum] = { -0.9961947f, -0.76604444f, -0.08715574f, 0.9961947f, 0.76604444f,0.08715574f };


	//! �r�̑�1�֐߂̏����p�x������[rad]�D���{�b�g�̐��ʂ� 0[rad]�Ƃ��āC�E�˂��𐳂ɂƂ�D
	constexpr static float DEFAULT_LEG_ANGLE[kLegNum] = { 
		::designlab::math_util::ConvertDegToRad(-45.0f),	::designlab::math_util::ConvertDegToRad(-90.0f),::designlab::math_util::ConvertDegToRad(-135.0f),
		::designlab::math_util::ConvertDegToRad(135.0f),	::designlab::math_util::ConvertDegToRad(90.0f),	::designlab::math_util::ConvertDegToRad(45.0f) 
	};

	//! DEFAULT_LEG_ANGLE�̒l������sin cos ���v�Z���Ă����D
	constexpr static float DEFAULT_LEG_ANGLE_SIN[kLegNum] = { -0.70710678118654f, -1.0f, -0.70710678118654f,
													 0.70710678118654f, 1.0f, 0.70710678118654f };

	constexpr static float DEFAULT_LEG_ANGLE_COS[kLegNum] = { 0.70710678118654f, 0.0f, -0.70710678118654f,
														 -0.70710678118654f, 0.0f, 0.70710678118654f };


};


#endif // !DESIGNLAB_HEXAPOD_CONST_H_