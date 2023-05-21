#include "LegState.h"
#include "HexapodConst.h"

bool LegState::isGrounded(const int _leg_state, const int _leg_num)
{
	int _shift_bit[HexapodConst::LEG_NUM] = { 0, 4, 8, 12, 16, 20 };//4bit�����炷���߂Ɏg�p����
	int v_bit = 0b1000;												//�ڒn�r������bit�̈ʒu�������ĂĂ���

	//_leg_num��0�`5�͈̔͂ɂ���K�v������̂�
	if (isAbleLegNum(_leg_num) == false) 
	{
		//�͈͊O�Ȃ��false���o�͂���
		return false; 
	}
	else 
	{
		//�w�肳�ꂽ�r�̐ڒn�r��bit�������Ă��邩���ׂ�
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

	//�r�̖{�������[�v����
	for (int i = 0; i < HexapodConst::LEG_NUM; i++)
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
	for (int i = 0; i < HexapodConst::LEG_NUM; i++)
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
	for (int i = 0; i < HexapodConst::LEG_NUM; i++)
	{
		if (isGrounded(_leg_state, i) == false)
		{
			//�����Ă���r�̋r�ԍ���vector�ɑ��
			_res_number.push_back(i);
		}
	}
}

int LegState::getLegState(const int _leg_state, const int _leg_num)
{
	const int _shift_bit[HexapodConst::LEG_NUM] = { 0, 4, 8, 12, 16, 20 };//4bit�����炷���߂Ɏg�p����
	const int kaisou_bit = 0b0111;	//�r�ʒu����������
	return ((_leg_state & (kaisou_bit << _shift_bit[_leg_num])) >> _shift_bit[_leg_num]);
}

int LegState::getComPatternState(const int _leg_state)
{
	//�d�S�p�^�[����ۑ�����r�b�g���}�X�N���C���̒l�����擾�ł���悤�ɉE�փV�t�g����D
	return ((_leg_state & COM_STATE_MASKBIT) >> SHIFT_TO_COM_NUM);
}

bool LegState::changeLegState(int& _leg_state, const int _leg_num, const int _new_state)
{
	//_leg_num �� _new_state �����������Ȃ�� false��Ԃ�
	if (isAbleLegNum(_leg_num) == false ||isAbleLegState(_new_state) == false ) 
	{
		return false;
	}

	//�V�����r��Ԃ𐶐�����
	int _mask = LEG_STATE_MASKBIT << (_leg_num * 4);	//4bit�̃f�[�^��ύX����n�_�܂Ń}�X�N�����炷
	int _state = _new_state << (_leg_num * 4);			//�r�ʒu�̃f�[�^��4bit�Âz�u����Ă���̂ł��̈ʒu�܂ňړ�����

	//�����Ă���r�̋r�ʒu�݂̂�ύX�i�r���I�_���a�ɂ�����r�b�g�̌��� https://qiita.com/vivisuke/items/bc707190e008551ca07f�j
	int _res = (_leg_state ^ _state) & _mask;
	_leg_state ^= _res;

	return true;
}

bool LegState::changeLegStateKeepTopBit(int& _leg_state, const int _leg_num, const int _new_state)
{
	//_leg_num �� _new_state �����������Ȃ�� false��Ԃ�
	if (isAbleLegNum(_leg_num) == false || isAbleLegState(_new_state) == false)
	{
		return false;
	}

	//�V�����r��Ԃ𐶐�����
	int _mask = LEG_POS_MASKBIT << (_leg_num * 4);	//4bit�̃f�[�^��ύX����n�_�܂Ń}�X�N�����炷
	int _state = _new_state << (_leg_num * 4);			//�r�ʒu�̃f�[�^��4bit�Âz�u����Ă���̂ł��̈ʒu�܂ňړ�����

	//�����Ă���r�̋r�ʒu�݂̂�ύX�i�r���I�_���a�ɂ�����r�b�g�̌��� https://qiita.com/vivisuke/items/bc707190e008551ca07f�j
	int _res = (_leg_state ^ _state) & _mask;
	_leg_state ^= _res;

	return true;
}

bool LegState::isAbleLegNum(const int _num)
{
	// 0 �` 5�Ȃ� true
	if (0 <= _num && _num < HexapodConst::LEG_NUM) { return true; }

	return false;
}

bool LegState::isAbleLegState(const int _state)
{
	// 8 (0b1000) �Ȃ� false
	if (_state == 8) { return false; }

	// 1 �` 15�Ȃ� true
	if (0 < _state && _state < 15) { return true; }

	return false;
}
