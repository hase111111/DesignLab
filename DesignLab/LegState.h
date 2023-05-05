#pragma once
#include <vector>

//���̃v���O�����ł͋r��Ԃ�int�^(32bit)�̏��ŕ\��
//���̃f�[�^���������邽�߂̊֐����܂Ƃ߂�����

namespace LegState
{
	//�r�ʒu��4bit�̉��ʎO���ŊǗ������̂ŁC�������}�X�N����
	const int LEG_POS_MASKBIT = 0b0111;

	//�r�ԍ�_leg_num 0�`5�ɉ����āC���̋r���ڒn���Ă��邩�𒲂ׂ�
	bool isGrounded(const int _leg_state, const int _leg_num);

	//�ڒn���Ă���r�̖{����Ԃ��֐�
	int getGroundedLegNum(const int _leg_state);

	//�ڒn���Ă���r�̋r�ԍ�0�`5���C����_res_number�ŎQ�Ɠn������֐�
	void getGroundedLegNumWithVector(const int _leg_state, std::vector<int>& _res_number);

	//�V�r���Ă���r�̋r�ԍ�0�`5���C����_res_number�ŎQ�Ɠn������֐�
	void getLiftedLegNumWithVector(const int _leg_state, std::vector<int>& _res_number);
}
