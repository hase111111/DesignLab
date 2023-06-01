#pragma once
#include <vector>

//���̃v���O�����ł͋r��Ԃ�int�^(32bit)�̏��ŕ\��
//���̃f�[�^���������邽�߂̊֐����܂Ƃ߂�����

//�r��Ԃ��Č����Ă邯�Ǐd�S�^�C�v�������Ă�DC++������int�^��32bit�D1�r�̋r��Ԃ�4bit�ŕ\�� �ŏ��0:�V�r,1:�ڒn�@
//�c��3bit�ŗ��U�������r�ʒu�D�r�͉E�O�r��0�Ƃ��Ď��v����0~5
// 
//	7   3    (0�͎g��Ȃ�)
//	6 4 2
//	5   1
// 
//	1111    1111		1111 1111 1111 1111 1111 1111
//	�]��    �d�S�p�^�[���r5  �r4  �r3  �r2  �r1  �r0

namespace LegState
{
	//�r��Ԃ̗��U�����DLegState.h�̒ʂ�7�ʂ�D
	constexpr int DISCRETE_NUM = 7;

	//�r�ʒu��4bit�̉��ʎO���ŊǗ������̂ŁC�������}�X�N����
	constexpr int LEG_POS_MASKBIT = 0b0111;

	//�r��Ԃ�4bit�ŊǗ������̂ŁC�������}�X�N����
	constexpr int LEG_STATE_MASKBIT = 0b1111;

	//�d�S�p�^�[����ۑ�����r�b�g�܂ōs�����߂ɁC�ǂꂾ���r�b�g���V�t�g���邩�D
	constexpr int SHIFT_TO_COM_NUM = 24;

	//�d�S�p�^�[����ۑ�����r�b�g���}�X�N����r�b�g�D
	constexpr int COM_STATE_MASKBIT = (0b1111 << SHIFT_TO_COM_NUM);

	//�r�ԍ�_leg_num 0�`5�ɉ����āC���̋r���ڒn���Ă��邩�𒲂ׂ�
	bool isGrounded(const int _leg_state, const int _leg_num);

	//�ڒn���Ă���r�̖{����Ԃ��֐�
	int getGroundedLegNum(const int _leg_state);

	//�ڒn���Ă���r�̋r�ԍ�0�`5���C����_res_number�ŎQ�Ɠn������֐�
	void getGroundedLegNumWithVector(const int _leg_state, std::vector<int>& _res_number);

	//�V�r���Ă���r�̋r�ԍ�0�`5���C����_res_number�ŎQ�Ɠn������֐�
	void getLiftedLegNumWithVector(const int _leg_state, std::vector<int>& _res_number);

	// �r��Ԃ��擾����D
	int getLegState(const int _leg_state, const int _leg_num);

	//�d�S�p�^�[�����擾����D0b0000 �` 0b1000 �܂�
	int getComPatternState(const int _leg_state);

	// �r�̏��� 0�`15 �̏�ԂɕύX����D�����̒l�����������Ȃ��false���o�͂���D
	// _leg_state �ύX����r��ԁD_leg_num �ǂ̋r��ύX���邩 0�`5�D_new_state �V�����r��� 1�`15
	bool changeLegState(int &_leg_state, const int _leg_num, const int _new_state);

	// �r�̏��� 0�`7 �̏�ԂɕύX����D�����̒l�����������Ȃ��false���o�͂���D�V�r��\��bit�͂��̂܂�
	// _leg_state �ύX����r��ԁD_leg_num �ǂ̋r��ύX���邩 0�`5�D_new_state �V�����r��� 1�`7
	bool changeLegStateKeepTopBit(int& _leg_state, const int _leg_num, const int _new_state);

	//�r�̐ڒn�E�V�r����ύX����D��1������_leg_state���Q�Ɠn�����ĕύX����
	void changeGround(int& _leg_state, const int _leg_num, const bool _ground);

	//�r�ԍ��� 0�`5 �̊Ԃɓ����Ă���Ȃ��true
	bool isAbleLegNum(const int _num);

	//�r��Ԃ� 1(0001)�`15(1111) �̊Ԃɓ����Ă���Ȃ��true
	bool isAbleLegState(const int _state);

	//�d�S�ʒu�ɂ����
}
