#include "LegState.h"

int LegStateEdit::makeLegState(const ComType::EComPattern _com_pattern, const bool _ground[HexapodConst::LEG_NUM], const int _leg_pos[HexapodConst::LEG_NUM])
{
	int res = 0;

	res |= ComType::convertComPatternToBit(_com_pattern) << SHIFT_TO_COM_NUM;	//�d�S�p�^�[���̐��l����bit�𗧂Ă�


	for (int i = 0; i < HexapodConst::LEG_NUM; i++)
	{
		if (_ground[i] == true) { res |= 0b1000 << (i * 4); }	//�ڒn���Ă���Ȃ�Ώ��bit�𗧂Ă�

		if (0 < _leg_pos[i] && _leg_pos[i] <= DISCRETE_NUM)
		{
			// 1 �` 7 �͈̔͂Ȃ�΂��̒l����bit�𗧂Ă�D
			res |= _leg_pos[i] << (i * 4);
		}
		else
		{
			//�͈͊O�Ȃ�� 4 (��ʒu)�ɂ���D
			res |= 4 << (i * 4);
		}
	}

	return res;
}

bool LegStateEdit::isGrounded(const int _leg_state, const int _leg_num)
{
	//_leg_num��0�`5�͈̔͂ɂ���K�v������̂ŁC�͈͊O�Ȃ��false���o�͂���
	if (isAbleLegNum(_leg_num) == false)
	{
		return false;
	}

	//�w�肳�ꂽ�r�̐ڒn�r��bit�������Ă��邩���ׂ�
	if ((_leg_state & (LEG_GROUNDED_MASKBIT << _leg_num * 4)))
	{
		return true;
	}
	else
	{
		return false;
	}
}

int LegStateEdit::getGroundedLegNum(const int _leg_state)
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

int LegStateEdit::getLiftedLegNum(const int _leg_state)
{
	return HexapodConst::LEG_NUM - getGroundedLegNum(_leg_state);
}

void LegStateEdit::getGroundedLegNumWithVector(const int _leg_state, std::vector<int>& _res_number)
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

void LegStateEdit::getLiftedLegNumWithVector(const int _leg_state, std::vector<int>& _res_number)
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

int LegStateEdit::getLegState(const int _leg_state, const int _leg_num)
{
	const int _shift_num = 4 * _leg_num;	//4bit�����炷

	return ((_leg_state & (LEG_POS_MASKBIT << _shift_num)) >> _shift_num);
}

int LegStateEdit::getComPatternState(const int _leg_state)
{
	//�d�S�p�^�[����ۑ�����r�b�g���}�X�N���C���̒l�����擾�ł���悤�ɉE�փV�t�g����D
	return ((_leg_state & COM_STATE_MASKBIT) >> SHIFT_TO_COM_NUM);
}

bool LegStateEdit::changeLegState(int& _leg_state, const int _leg_num, const int _new_state)
{
	//_leg_num �� _new_state �����������Ȃ�� false��Ԃ�
	if (isAbleLegNum(_leg_num) == false || isAbleLegState(_new_state) == false)
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

bool LegStateEdit::changeLegStateKeepTopBit(int& _leg_state, const int _leg_num, const int _new_state)
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

void LegStateEdit::changeGround(int& _leg_state, const int _leg_num, const bool _ground)
{
	//_leg_num �����������Ȃ�΁C�I���D
	if (isAbleLegNum(_leg_num) == false) { return; }

	if (_ground == true)
	{
		_leg_state |= 0b1000 << (_leg_num * 4);
	}
	else
	{
		_leg_state &= ~(0b1000 << (_leg_num * 4));
	}
}
