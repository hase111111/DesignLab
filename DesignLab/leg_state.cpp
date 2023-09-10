#include "leg_state.h"


int dl_leg::makeLegState(const ComType::EComPattern _com_pattern, const bool _ground[HexapodConst::LEG_NUM], const int _leg_pos[HexapodConst::LEG_NUM])
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


bool dl_leg::isGrounded(const int _leg_state, const int _leg_num)
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


int dl_leg::getGroundedLegNum(const int leg_state)
{
	int res = 0;

	//�r�̖{�������[�v����
	for (int i = 0; i < HexapodConst::LEG_NUM; i++)
	{
		if (isGrounded(leg_state, i) == true)
		{
			//�ڒn���Ă���r������΃J�E���g�A�b�v����
			res++;
		}
	}

	return res;
}


int dl_leg::getLiftedLegNum(const int _leg_state)
{
	return HexapodConst::LEG_NUM - getGroundedLegNum(_leg_state);
}


void dl_leg::getGroundedLegNumWithVector(const int leg_state, std::vector<int>* res_index)
{
	(*res_index).clear();

	//�r��6�{����̂�6�񃋁[�v����
	for (int i = 0; i < HexapodConst::LEG_NUM; i++)
	{
		if (isGrounded(leg_state, i))
		{
			//�ڒn���Ă���r�̋r�ԍ���vector�ɑ��
			(*res_index).push_back(i);
		}
	}
}


void dl_leg::getLiftedLegNumWithVector(const int _leg_state, std::vector<int>* res_index)
{
	(*res_index).clear();

	//�r��6�{����̂�6�񃋁[�v����
	for (int i = 0; i < HexapodConst::LEG_NUM; i++)
	{
		if (!isGrounded(_leg_state, i))
		{
			//�����Ă���r�̋r�ԍ���vector�ɑ��
			(*res_index).push_back(i);
		}
	}
}


int dl_leg::getLegState(const int _leg_state, const int _leg_num)
{
	const int _shift_num = 4 * _leg_num;	//4bit�����炷

	return ((_leg_state & (LEG_POS_MASKBIT << _shift_num)) >> _shift_num);
}


int dl_leg::getComPatternState(const int _leg_state)
{
	//�d�S�p�^�[����ۑ�����r�b�g���}�X�N���C���̒l�����擾�ł���悤�ɉE�փV�t�g����D
	return ((_leg_state & COM_STATE_MASKBIT) >> SHIFT_TO_COM_NUM);
}


bool dl_leg::changeLegState(const int leg_index, const int new_discretized_leg_pos, int* leg_state)
{
	//_leg_num �� _new_state �����������Ȃ�� false��Ԃ�
	if (!isAbleLegNum(leg_index) || !isAbleLegState(new_discretized_leg_pos))
	{
		return false;
	}

	//�V�����r��Ԃ𐶐�����
	int mask = LEG_STATE_MASKBIT << (leg_index * 4);	//4bit�̃f�[�^��ύX����n�_�܂Ń}�X�N�����炷
	int state = new_discretized_leg_pos << (leg_index * 4);			//�r�ʒu�̃f�[�^��4bit�Âz�u����Ă���̂ł��̈ʒu�܂ňړ�����

	//�����Ă���r�̋r�ʒu�݂̂�ύX�i�r���I�_���a�ɂ�����r�b�g�̌��� https://qiita.com/vivisuke/items/bc707190e008551ca07f�j
	int res = ((*leg_state) ^ state) & mask;
	(*leg_state) ^= res;

	return true;
}


bool dl_leg::changeLegStateKeepTopBit(int& _leg_state, const int _leg_num, const int _new_state)
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


void dl_leg::changeGround(int& _leg_state, const int _leg_num, const bool _ground)
{
	//_leg_num �����������Ȃ�΁C�I���D
	if (isAbleLegNum(_leg_num) == false) { return; }

	if (_ground == true)
	{
		_leg_state |= (0b1000 << (_leg_num * 4));
	}
	else
	{
		_leg_state &= ~(0b1000 << (_leg_num * 4));
	}
}


int dl_leg::changeComPattern(int leg_state, const ComType::EComPattern new_com_pattern)
{
	const int state = ComType::convertComPatternToBit(new_com_pattern) << SHIFT_TO_COM_NUM;
	int res = (leg_state ^ state) & COM_STATE_MASKBIT;
	leg_state ^= res;

	return leg_state;
}


int dl_leg::getLegUpDownCount(const int _leg_state_first, const int _leg_state_second)
{
	int _res = 0;

	for (int i = 0; i < HexapodConst::LEG_NUM; i++)
	{
		int _first_state = _leg_state_first & (LEG_GROUNDED_MASKBIT << (i * 4));
		int _second_state = _leg_state_second & (LEG_GROUNDED_MASKBIT << (i * 4));

		if (_first_state ^ _second_state)
		{
			_res++;
		}
	}

	return _res;
}
