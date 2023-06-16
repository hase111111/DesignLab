#include "LegState.h"
#include "ComType.h"

int leg_state::makeLegState(const int _com_pattern, const bool _ground[HexapodConst::LEG_NUM], const int _leg_pos[HexapodConst::LEG_NUM])
{
	int res = 0;

	if (0 <= _com_pattern && _com_pattern < ComType::COM_PATTERN_NUM) 
	{
		res |= _com_pattern << SHIFT_TO_COM_NUM;	//�d�S�p�^�[���̐��l����bit�𗧂Ă�
	}

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

bool leg_state::isGrounded(const int _leg_state, const int _leg_num)
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

int leg_state::getGroundedLegNum(const int _leg_state)
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

void leg_state::getGroundedLegNumWithVector(const int _leg_state, std::vector<int>& _res_number)
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

void leg_state::getLiftedLegNumWithVector(const int _leg_state, std::vector<int>& _res_number)
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

int leg_state::getLegState(const int _leg_state, const int _leg_num)
{
	const int _shift_bit[HexapodConst::LEG_NUM] = { 0, 4, 8, 12, 16, 20 };//4bit�����炷���߂Ɏg�p����
	const int kaisou_bit = 0b0111;	//�r�ʒu����������
	return ((_leg_state & (kaisou_bit << _shift_bit[_leg_num])) >> _shift_bit[_leg_num]);
}

int leg_state::getComPatternState(const int _leg_state)
{
	//�d�S�p�^�[����ۑ�����r�b�g���}�X�N���C���̒l�����擾�ł���悤�ɉE�փV�t�g����D
	return ((_leg_state & COM_STATE_MASKBIT) >> SHIFT_TO_COM_NUM);
}

bool leg_state::changeLegState(int& _leg_state, const int _leg_num, const int _new_state)
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

bool leg_state::changeLegStateKeepTopBit(int& _leg_state, const int _leg_num, const int _new_state)
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

void leg_state::changeGround(int& _leg_state, const int _leg_num, const bool _ground)
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
