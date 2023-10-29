#include "leg_state.h"

#include "cassert_define.h"


designlab::leg_func::LegStateBit designlab::leg_func::MakeLegStateBit(
	const DiscreteComPos discrete_com_pos, 
	const std::array<bool, HexapodConst::kLegNum>& is_ground,
	const std::array<DiscreteLegPos, HexapodConst::kLegNum>& discretized_leg_pos
)
{
	LegStateBit res = 0;

	LegStateBit discrete_com_pos_bit = static_cast<unsigned int>(discrete_com_pos);		//bit�ɕϊ�����
	res |= discrete_com_pos_bit << kShiftToComNum;	//�d�S�p�^�[���̐��l����bit�𗧂Ă�


	for (int i = 0; i < HexapodConst::kLegNum; i++)
	{
		//�ڒn���Ă���Ȃ�Ώ��bit�𗧂Ă�
		size_t ground_bit_index = static_cast<size_t>(i + 1) * 4 - 1;
		if (is_ground[i]) { res[ground_bit_index] = true; }

		// �r��bit�𗧂Ă�
		LegStateBit discrete_leg_pos_bit = static_cast<unsigned int>(discretized_leg_pos[i]);	//bit�ɕϊ�����
		size_t shift_num = static_cast<size_t>(i) * 4;	//4bit�����炷

		res |= discrete_leg_pos_bit << shift_num;	//�r�̈ʒu�̐��l����bit�𗧂Ă�
	}

	return res;
}


bool designlab::leg_func::IsGrounded(const LegStateBit& leg_state, const int leg_index)
{
	// leg_index��0�`5�͈̔͂ɂ���K�v������D
	assert(0 <= leg_index);
	assert(leg_index < HexapodConst::kLegNum);

	//�w�肳�ꂽ�r�̐ڒn�r��bit�������Ă��邩���ׂ�
	size_t ground_bit_index = static_cast<size_t>(leg_index + 1) * 4 - 1;

	if (leg_state[ground_bit_index])
	{
		return true;
	}
	else
	{
		return false;
	}
}

designlab::leg_func::LegGroundedBit designlab::leg_func::GetLegGroundedBit(const LegStateBit& leg_state)
{
	LegGroundedBit res;

	for (int i = 0; i < HexapodConst::kLegNum; i++)
	{
		if (IsGrounded(leg_state, i))
		{
			res[i] = true;
		}
		else
		{
			res[i] = false;
		}
	}

	return res;
}

int designlab::leg_func::GetGroundedLegNum(const LegStateBit& leg_state)
{
	int res = 0;

	//�r�̖{�������[�v����
	for (int i = 0; i < HexapodConst::kLegNum; i++)
	{
		if (IsGrounded(leg_state, i))
		{
			//�ڒn���Ă���r������΃J�E���g�A�b�v����
			res++;
		}
	}

	return res;
}

int designlab::leg_func::GetLiftedLegNum(const LegStateBit& leg_state)
{
	return HexapodConst::kLegNum - GetGroundedLegNum(leg_state);
}

void designlab::leg_func::GetGroundedLegIndexByVector(const LegStateBit& leg_state, std::vector<int>* res_index)
{
	// res_index��nullptr�łȂ����ƁC����ł���K�v������
	assert(res_index != nullptr);
	assert((*res_index).size() == 0);

	//�r��6�{����̂�6�񃋁[�v����
	for (int i = 0; i < HexapodConst::kLegNum; i++)
	{
		if (IsGrounded(leg_state, i))
		{
			//�ڒn���Ă���r�̋r�ԍ���vector�ɑ��
			(*res_index).push_back(i);
		}
	}
}

void designlab::leg_func::GetLiftedLegIndexByVector(const LegStateBit& leg_state, std::vector<int>* res_index)
{
	// res_index��nullptr�łȂ����ƁC����ł���K�v������
	assert(res_index != nullptr);
	assert((*res_index).size() == 0);

	//�r��6�{����̂�6�񃋁[�v����
	for (int i = 0; i < HexapodConst::kLegNum; i++)
	{
		if (!IsGrounded(leg_state, i))
		{
			//�����Ă���r�̋r�ԍ���vector�ɑ��
			(*res_index).push_back(i);
		}
	}
}

DiscreteLegPos designlab::leg_func::GetDiscreteLegPos(const LegStateBit& leg_state, const int leg_index)
{
	// leg_index��0�`5�͈̔͂ɂ���K�v������D
	assert(0 <= leg_index);
	assert(leg_index < HexapodConst::kLegNum);

	const int shift_num = 4 * leg_index;	//4bit�����炷

	const int res = static_cast<int>(((leg_state & (kLegPosMaskbit << shift_num)) >> shift_num).to_ulong());

	return static_cast<DiscreteLegPos>(res);
}

DiscreteComPos designlab::leg_func::GetDiscreteComPos(const LegStateBit& leg_state)
{
	//�d�S�p�^�[����ۑ�����r�b�g���}�X�N���C���̒l�����擾�ł���悤�ɉE�փV�t�g����D
	const int res = static_cast<int>(((leg_state & kComStateMaskbit) >> kShiftToComNum).to_ulong());

	return static_cast<DiscreteComPos>(res);
}


void designlab::leg_func::ChangeLegState(
	const int leg_index, 
	const DiscreteLegPos new_discretized_leg_pos,
	const bool is_ground,
	LegStateBit* leg_state
)
{
	// leg_index��0�`5�͈̔͂ɂ���K�v������D
	assert(0 <= leg_index);
	assert(leg_index < HexapodConst::kLegNum);

	// leg_state �� nullptr�ł͂Ȃ�
	assert(leg_state != nullptr);

	ChangeDiscreteLegPos(leg_index, new_discretized_leg_pos, leg_state);
	ChangeGround(leg_index, is_ground, leg_state);
}

void designlab::leg_func::ChangeDiscreteLegPos(
	const int leg_index, 
	const DiscreteLegPos new_discretized_leg_pos,
	LegStateBit* leg_state
)
{
	// leg_index��0�`5�͈̔͂ɂ���K�v������D
	assert(0 <= leg_index);
	assert(leg_index < HexapodConst::kLegNum);

	// leg_state �� nullptr�ł͂Ȃ�
	assert(leg_state != nullptr);


	//�V�����r��Ԃ𐶐�����
	const size_t shift_num = static_cast<size_t>(leg_index) * 4;									//4bit�����炷
	const LegStateBit mask = kLegPosMaskbit << shift_num;											//4bit�̃f�[�^��ύX����n�_�܂Ń}�X�N�����炷
	const LegStateBit discreate_leg_pos_bit = static_cast<unsigned int>(new_discretized_leg_pos);	//bit�ɕϊ�����
	const LegStateBit state = discreate_leg_pos_bit << shift_num;	//�r�ʒu�̃f�[�^��4bit�Âz�u����Ă���̂ł��̈ʒu�܂ňړ�����

	//�����Ă���r�̋r�ʒu�݂̂�ύX�i�r���I�_���a�ɂ�����r�b�g�̌��� https://qiita.com/vivisuke/items/bc707190e008551ca07f�j
	LegStateBit res = ((*leg_state) ^ state) & mask;
	(*leg_state) ^= res;
}

void designlab::leg_func::ChangeGround(const int leg_index, const bool is_ground, LegStateBit* leg_state)
{
	// leg_index��0�`5�͈̔͂ɂ���K�v������D
	assert(0 <= leg_index);
	assert(leg_index < HexapodConst::kLegNum);

	// leg_state �� nullptr�ł͂Ȃ�
	assert(leg_state != nullptr);


	//�w�肳�ꂽ�r�̐ڒn�r��bit�𗧂Ă邩����������
	const size_t ground_bit_index = static_cast<size_t>(leg_index + 1) * 4 - 1;

	if (is_ground)
	{
		(*leg_state)[ground_bit_index] = true;
	}
	else
	{
		(*leg_state)[ground_bit_index] = false;
	}
}

void designlab::leg_func::ChangeAllLegGround(const LegGroundedBit& is_ground_list, LegStateBit* leg_state)
{
	// leg_state �� nullptr�ł͂Ȃ�
	assert(leg_state != nullptr);

	for (int i = 0; i < HexapodConst::kLegNum; i++)
	{
		ChangeGround(i, is_ground_list[i], leg_state);
	}
}

void designlab::leg_func::ChangeDiscreteComPos(const DiscreteComPos new_com_pattern, LegStateBit* leg_state)
{
	// leg_state �� nullptr�ł͂Ȃ�
	assert(leg_state != nullptr);

	const LegStateBit state = static_cast<unsigned int>(new_com_pattern) << kShiftToComNum;
	LegStateBit sub = ((*leg_state) ^ state) & kComStateMaskbit;
	(*leg_state) ^= sub;
}