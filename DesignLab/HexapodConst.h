#pragma once

// Hexapod�CphantomX�̃p�����[�^��萔�ŕ\���������́D

class HexapodConst final
{
public:

	//Hexapod�̋r�̖{����\���D�����ύX���Ă��r�̖{�����ύX�ł���킯�ł͂Ȃ��D�}�W�b�N�i���o�[���Ȃ������Ƃ��ړI�D
	constexpr static int LEG_NUM = 6;

	//���{�b�g�̐��@
	const static float COXA_LENGTH;			//��1�֐ߕ��̒����D�ڂ�����reference���t�H���_�Q�ƁD
	const static float FEMUR_LENGTH;		//��2�֐ߕ��̒����D�ڂ�����reference���t�H���_�Q�ƁD
	const static float TIBIA_LENGTH;		//��3�֐ߕ��̒����D�ڂ�����reference���t�H���_�Q�ƁD
	const static float BODY_FRONT_WIDTH;	//�O���̕�[mm]�DphantomX�̉���.
	const static float BODY_CENTER_WIDTH;	//���S�̕�[mm]�DphantomX�̉���.
	const static float BODY_REAR_WIDTH;		//����̕�[mm]�DphantomX�̉���.
	const static float BODY_FRONT_LENGTH;	//��������O���܂ł̋���[mm]�DphantomX�̏c��.
	const static float BODY_REAR_LENGTH;	//��������O���܂ł̋���[mm]�DphantomX�̏c��.
	const static float BODY_HEIGHT;			//���̂̍���[mm]�D

	//���{�b�g�̉��͈�
	const static float VERTICAL_MAX_RANGE;			//�n�ʂ̍ō��_�Ɠ��̉����̌��Ԃ̍ő�l�������D�r��L�΂��؂�Ȃ����x�ɐݒ肷��D���� MAX_DELTAZ
	const static float VERTICAL_MIN_RANGE;			//�n�ʂ̍ō��_�Ɠ��̉����̌��Ԃ̍ŏ��l�������D���� MIN_DELTAZ
	const static float MOVABLE_LEG_RANGE;			//coxa�֐�(��1�֐�)�̉ғ��\�Ȋp�x [rad]
	const static float DEFAULT_LEG_ANGLE[LEG_NUM];	//�r�̑�1�֐߂̏����p�x������[rad]�D���{�b�g�̐��ʂ� 0[rad]�Ƃ��āC�E�˂��𐳂ɂƂ�D

private:
	//�R���X�g���N�^�ƃR�s�[�R���X�g���N�^���폜���C���̂𐶐��ł��Ȃ��悤�ɂ���D
	HexapodConst() = delete;
	HexapodConst(const HexapodConst& _other) = delete;
};

