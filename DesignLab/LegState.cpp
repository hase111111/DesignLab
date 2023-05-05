#include "LegState.h"

bool LegState::isGrounded(const int _leg_state, const int _leg_num)
{
	int _shift_bit[6] = { 0, 4, 8, 12, 16, 20 };//4bit�����炷���߂Ɏg�p����
	int v_bit = 0b1000;							//�V�r������bit�̈ʒu�������ĂĂ���

	//_leg_num��0�`5�͈̔͂ɂ���K�v������̂�
	if (_leg_num < 0 || _leg_num > 5) 
	{
		//�͈͊O�Ȃ��false���o�͂���
		return false; 
	}
	else 
	{
		//�w�肳�ꂽ�r�̗V�r��bit�������Ă��邩���ׂ�
		if ((_leg_state & (v_bit << _shift_bit[_leg_num])))
		{
			return true;
		}
		else
		{
			return false;
		}
	}
	
}

int LegState::getGroundedLegNum(const int _leg_state)
{
	int _res = 0;

	//�r��6�{����̂�6�񃋁[�v����
	for (int i = 0; i < 6; i++)
	{
		if (isGrounded(_leg_state, i) == true)
		{
			//�ڒn���Ă���r������΃J�E���g�A�b�v����
			_res++;
		}
	}

	return _res;
}

void LegState::getGroundedLegNumWithVector(const int _leg_state, std::vector<int>& _res_number)
{
	//�r��6�{����̂�6�񃋁[�v����
	for (int i = 0; i < 6; i++)
	{
		if (isGrounded(_leg_state, i) == true)
		{
			//�ڒn���Ă���r�̋r�ԍ���vector�ɑ��
			_res_number.push_back(i);
		}
	}
}

void LegState::getLiftedLegNumWithVector(const int _leg_state, std::vector<int>& _res_number)
{
	//�r��6�{����̂�6�񃋁[�v����
	for (int i = 0; i < 6; i++)
	{
		if (isGrounded(_leg_state, i) == false)
		{
			//�����Ă���r�̋r�ԍ���vector�ɑ��
			_res_number.push_back(i);
		}
	}
}
