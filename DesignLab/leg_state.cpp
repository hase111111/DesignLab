#include "leg_state.h"


std::bitset<dl_leg::LEG_STATE_BIT_NUM> dl_leg::makeLegState(const EDiscreteComPos com_pattern,
	const bool is_ground[HexapodConst::LEG_NUM], const EDiscreteLegPos discretized_leg_pos[HexapodConst::LEG_NUM])
{
	std::bitset<LEG_STATE_BIT_NUM> res = 0;

	res |= static_cast<int>(com_pattern) << SHIFT_TO_COM_NUM;	//�d�S�p�^�[���̐��l����bit�𗧂Ă�


	for (int i = 0; i < HexapodConst::LEG_NUM; i++)
	{
		//�ڒn���Ă���Ȃ�Ώ��bit�𗧂Ă�
		if (is_ground[i]) { res[(i + 1) * 4 - 1] = true; }

		// �r��bit�𗧂Ă�
		res |= static_cast<int>(discretized_leg_pos[i]) << (i * 4);
	}

	return res;
}


bool dl_leg::isGrounded(const std::bitset<LEG_STATE_BIT_NUM>& leg_state, const int leg_index)
{
	//_leg_num��0�`5�͈̔͂ɂ���K�v������̂ŁC�͈͊O�Ȃ��false���o�͂���
	if (!isAbleLegNum(leg_index))
	{
		return false;
	}

	//�w�肳�ꂽ�r�̐ڒn�r��bit�������Ă��邩���ׂ�
	if (leg_state[(leg_index + 1) * 4 - 1])
	{
		return true;
	}
	else
	{
		return false;
	}
}


int dl_leg::getGroundedLegNum(const std::bitset<LEG_STATE_BIT_NUM>& leg_state)
{
	int res = 0;

	//�r�̖{�������[�v����
	for (int i = 0; i < HexapodConst::LEG_NUM; i++)
	{
		if (isGrounded(leg_state, i))
		{
			//�ڒn���Ă���r������΃J�E���g�A�b�v����
			res++;
		}
	}

	return res;
}


int dl_leg::getLiftedLegNum(const std::bitset<LEG_STATE_BIT_NUM>& leg_state)
{
	return HexapodConst::LEG_NUM - getGroundedLegNum(leg_state);
}


void dl_leg::getGroundedLegIndexWithVector(const std::bitset<LEG_STATE_BIT_NUM>& leg_state, std::vector<int>* res_index)
{
	if (res_index == nullptr) { return; }

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


void dl_leg::getLiftedLegIndexWithVector(const std::bitset<LEG_STATE_BIT_NUM>& leg_state, std::vector<int>* res_index)
{
	if (res_index == nullptr) { return; }

	(*res_index).clear();

	//�r��6�{����̂�6�񃋁[�v����
	for (int i = 0; i < HexapodConst::LEG_NUM; i++)
	{
		if (!isGrounded(leg_state, i))
		{
			//�����Ă���r�̋r�ԍ���vector�ɑ��
			(*res_index).push_back(i);
		}
	}
}


EDiscreteLegPos dl_leg::getLegState(const std::bitset<dl_leg::LEG_STATE_BIT_NUM>& leg_state, const int leg_index)
{
	const int shift_num = 4 * leg_index;	//4bit�����炷

	const int res = static_cast<int>(((leg_state & (LEG_POS_MASKBIT << shift_num)) >> shift_num).to_ulong());

	return static_cast<EDiscreteLegPos>(res);
}


EDiscreteComPos dl_leg::getComPatternState(const std::bitset<dl_leg::LEG_STATE_BIT_NUM>& leg_state)
{
	//�d�S�p�^�[����ۑ�����r�b�g���}�X�N���C���̒l�����擾�ł���悤�ɉE�փV�t�g����D
	const int res = static_cast<int>(((leg_state & COM_STATE_MASKBIT) >> SHIFT_TO_COM_NUM).to_ulong());

	return static_cast<EDiscreteComPos>(res);
}


bool dl_leg::changeLegState(int leg_index, EDiscreteLegPos new_discretized_leg_pos, bool is_ground, std::bitset<dl_leg::LEG_STATE_BIT_NUM>* leg_state)
{
	//leg_num �� _new_state �����������Ȃ�� false��Ԃ�
	if (!isAbleLegNum(leg_index) || leg_state == nullptr)
	{
		return false;
	}

	//�V�����r��Ԃ𐶐�����
	std::bitset<dl_leg::LEG_STATE_BIT_NUM> mask = LEG_STATE_MASKBIT << (leg_index * 4);								//4bit�̃f�[�^��ύX����n�_�܂Ń}�X�N�����炷
	std::bitset<dl_leg::LEG_STATE_BIT_NUM> state = static_cast<int>(new_discretized_leg_pos) << (leg_index * 4);	//�r�ʒu�̃f�[�^��4bit�Âz�u����Ă���̂ł��̈ʒu�܂ňړ�����

	//�����Ă���r�̋r�ʒu�݂̂�ύX�i�r���I�_���a�ɂ�����r�b�g�̌��� https://qiita.com/vivisuke/items/bc707190e008551ca07f�j
	std::bitset<dl_leg::LEG_STATE_BIT_NUM> res = ((*leg_state) ^ state) & mask;
	(*leg_state) ^= res;

	return true;
}


bool dl_leg::changeLegStateKeepTopBit(const int leg_index, const EDiscreteLegPos new_discretized_leg_pos, std::bitset<dl_leg::LEG_STATE_BIT_NUM>* leg_state)
{
	//leg_num �� _new_state �����������Ȃ�� false��Ԃ�
	if (!isAbleLegNum(leg_index) || leg_state == nullptr)
	{
		return false;
	}

	//�V�����r��Ԃ𐶐�����
	std::bitset<LEG_STATE_BIT_NUM> mask = LEG_POS_MASKBIT << (leg_index * 4);								//4bit�̃f�[�^��ύX����n�_�܂Ń}�X�N�����炷
	std::bitset<LEG_STATE_BIT_NUM> state = static_cast<int>(new_discretized_leg_pos) << (leg_index * 4);	//�r�ʒu�̃f�[�^��4bit�Âz�u����Ă���̂ł��̈ʒu�܂ňړ�����

	//�����Ă���r�̋r�ʒu�݂̂�ύX�i�r���I�_���a�ɂ�����r�b�g�̌��� https://qiita.com/vivisuke/items/bc707190e008551ca07f�j
	std::bitset<LEG_STATE_BIT_NUM> res = ((*leg_state) ^ state) & mask;
	(*leg_state) ^= res;

	return true;
}


void dl_leg::changeGround(const int leg_index, const bool is_ground, std::bitset<dl_leg::LEG_STATE_BIT_NUM>* leg_state)
{
	//leg_num �����������Ȃ�΁C�I���D
	if (!isAbleLegNum(leg_index)) { return; }


	//�w�肳�ꂽ�r�̐ڒn�r��bit�𗧂Ă邩����������
	if (is_ground)
	{
		(*leg_state)[(leg_index + 1) * 4 - 1] = true;
	}
	else
	{
		(*leg_state)[(leg_index + 1) * 4 - 1] = false;
	}
}


std::bitset < dl_leg::LEG_STATE_BIT_NUM > dl_leg::changeComPattern(const std::bitset<LEG_STATE_BIT_NUM>& leg_state, const EDiscreteComPos new_com_pattern)
{
	std::bitset < dl_leg::LEG_STATE_BIT_NUM > res = leg_state;

	const std::bitset < dl_leg::LEG_STATE_BIT_NUM > state = static_cast<int>(new_com_pattern) << SHIFT_TO_COM_NUM;
	std::bitset < dl_leg::LEG_STATE_BIT_NUM > sub = (res ^ state) & COM_STATE_MASKBIT;
	res ^= sub;

	return res;
}


int dl_leg::getLegUpDownCount(const int _leg_state_first, const int _leg_state_second)
{
	int res = 0;

	//for (int i = 0; i < HexapodConst::LEG_NUM; i++)
	//{
	//	int first_state = _leg_state_first & (LEG_GROUNDED_MASKBIT << (i * 4));
	//	int second_state = _leg_state_second & (LEG_GROUNDED_MASKBIT << (i * 4));

	//	if (first_state ^ second_state)
	//	{
	//		res++;
	//	}
	//}

	return res;
}