#pragma once

// Hexapod�CphantomX�̃p�����[�^��萔�ŕ\���������́D

class HexapodConst final
{
public:
	//�R���X�g���N�^�ƃR�s�[�R���X�g���N�^���폜���C���̂𐶐��ł��Ȃ��悤�ɂ���D
	HexapodConst() = delete;
	HexapodConst(const HexapodConst& _other) = delete;

	//Hexapod�̋r�̖{����\���D�����ύX���Ă��r�̖{�����ύX�ł���킯�ł͂Ȃ��D�}�W�b�N�i���o�[���Ȃ������Ƃ��ړI�D
	constexpr static int LEG_NUM = 6;

	//���{�b�g�̐��@
	const static double COXA_LENGTH;		//��1�֐ߕ��̒����D�ڂ�����reference���t�H���_�Q�ƁD
	const static double FEMUR_LENGTH;		//��2�֐ߕ��̒����D�ڂ�����reference���t�H���_�Q�ƁD
	const static double TIBIA_LENGTH;		//��3�֐ߕ��̒����D�ڂ�����reference���t�H���_�Q�ƁD
	const static double BODY_FRONT_WIDTH;	//�O���̕�[mm]�DphantomX�̉���.
	const static double BODY_CENTER_WIDTH;	//���S�̕�[mm]�DphantomX�̉���.
	const static double BODY_REAR_WIDTH;	//����̕�[mm]�DphantomX�̉���.
	const static double BODY_FRONT_LENGTH;	//��������O���܂ł̋���[mm]�DphantomX�̏c��.
	const static double BODY_REAR_LENGTH;	//��������O���܂ł̋���[mm]�DphantomX�̏c��.

	//���{�b�g�̉��͈�
	const static double VERTICAL_MAX_RANGE;	//�n�ʂ̍ō��_�Ɠ��̉����̌��Ԃ̍ő�l�������D�r��L�΂��؂�Ȃ����x�ɐݒ肷��D���� MAX_DELTAZ
	const static double VERTICAL_MIN_RANGE;	//�n�ʂ̍ō��_�Ɠ��̉����̌��Ԃ̍ŏ��l�������D���� MIN_DELTAZ
};

