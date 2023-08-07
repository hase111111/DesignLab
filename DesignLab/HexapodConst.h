#pragma once
#include "my_math.h"

//! @brief  Hexapod�CphantomX�̃p�����[�^��萔�ŕ\���������́D
class HexapodConst final
{
public:

	//Hexapod�̋r�̖{����\���D�����ύX���Ă��r�̖{�����ύX�ł���킯�ł͂Ȃ��D�}�W�b�N�i���o�[���Ȃ������Ƃ��ړI�D
	constexpr static int LEG_NUM = 6;

	//���{�b�g�̐��@
	constexpr static float COXA_LENGTH = 52.0f;			//!< ��1�֐ߕ��̒����D�ڂ�����reference���t�H���_�Q�ƁD
	constexpr static float FEMUR_LENGTH = 66.0f;		//��2�֐ߕ��̒����D�ڂ�����reference���t�H���_�Q�ƁD
	constexpr static float TIBIA_LENGTH = 130.0f;		//��3�֐ߕ��̒����D�ڂ�����reference���t�H���_�Q�ƁD
	constexpr static float BODY_FRONT_WIDTH = 60.0f;	//�O���̕�[mm]�DphantomX�̉���.
	constexpr static float BODY_CENTER_WIDTH = 100.0f;	//���S�̕�[mm]�DphantomX�̉���.
	constexpr static float BODY_REAR_WIDTH = 60.0f;		//����̕�[mm]�DphantomX�̉���.
	constexpr static float BODY_FRONT_LENGTH = 120.0f;	//��������O���܂ł̋���[mm]�DphantomX�̏c��.
	constexpr static float BODY_REAR_LENGTH = 120.0f;	//��������O���܂ł̋���[mm]�DphantomX�̏c��.
	constexpr static float BODY_HEIGHT = 40.0f;			//���̂̍���[mm]�D

	//���{�b�g�̉��͈�
	const static float VERTICAL_MAX_RANGE;			//�n�ʂ̍ō��_�Ɠ��̉����̌��Ԃ̍ő�l�������D�r��L�΂��؂�Ȃ����x�ɐݒ肷��D���� MAX_DELTAZ
	const static float VERTICAL_MIN_RANGE;			//�n�ʂ̍ō��_�Ɠ��̉����̌��Ԃ̍ŏ��l�������D���� MIN_DELTAZ

	constexpr static float MOVABLE_LEG_RANGE = my_math::convertDegToRad(40.0f);			//coxa�֐�(��1�֐�)�̉ғ��\�Ȋp�x [rad]

	//MOVABLE_LEG_RANGE��cos�l -85���C-130���C-175���C95���C50���C5��
	constexpr static float MOVABLE_LEG_RANGE_COS_MAX[LEG_NUM] = { 0.08715574f , -0.6427876f, -0.9961947f, -0.08715574f, 0.6427876f, 0.9961947f };

	//MOVABLE_LEG_RANGE��cos�l -5���C-50���C-95���C175���C130���C85��
	constexpr static float MOVABLE_LEG_RANGE_COS_MIN[LEG_NUM] = { 0.9961947f,  0.6427876f, -0.08715574f, -0.9961947f, -0.6427876f, 0.08715574f };

	//MOVABLE_LEG_RANGE��sin�l -5���C-50���C-95���C175���C130���C85��
	constexpr static float MOVABLE_LEG_RANGE_SIN_MAX[LEG_NUM] = { -0.08715574f, -0.76604444f, -0.9961947f, 0.0871557f, 0.76604444f, 0.9961946f };

	//MOVABLE_LEG_RANGE��sin�l -85���C-130���C-175���C95���C50���C5��
	constexpr static float MOVABLE_LEG_RANGE_SIN_MIN[LEG_NUM] = { -0.9961947f, -0.76604444f, -0.08715574f, 0.9961947f, 0.76604444f,0.08715574f };


	//! �r�̑�1�֐߂̏����p�x������[rad]�D���{�b�g�̐��ʂ� 0[rad]�Ƃ��āC�E�˂��𐳂ɂƂ�D
	constexpr static float DEFAULT_LEG_ANGLE[LEG_NUM] = { my_math::convertDegToRad(-45.0f),	my_math::convertDegToRad(-90.0f),	my_math::convertDegToRad(-135.0f),
													my_math::convertDegToRad(135.0f),	my_math::convertDegToRad(90.0f),	my_math::convertDegToRad(45.0f) };

	//! DEFAULT_LEG_ANGLE�̒l������sin cos ���v�Z���Ă����D
	constexpr static float DEFAULT_LEG_ANGLE_SIN[LEG_NUM] = { -0.70710678118654f, -1.0f, -0.70710678118654f,
													 0.70710678118654f, 1.0f, 0.70710678118654f };

	constexpr static float DEFAULT_LEG_ANGLE_COS[LEG_NUM] = { 0.70710678118654f, 0.0f, -0.70710678118654f,
														 -0.70710678118654f, 0.0f, 0.70710678118654f };

private:
	//�R���X�g���N�^�ƃR�s�[�R���X�g���N�^���폜���C���̂𐶐��ł��Ȃ��悤�ɂ���D
	HexapodConst() = delete;
	HexapodConst(const HexapodConst& _other) = delete;
};

