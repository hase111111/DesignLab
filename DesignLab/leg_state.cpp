#include "leg_state.h"

#include "cassert_define.h"


namespace designlab::leg_func
{
	LegStateBit MakeLegStateBit(const DiscreteComPos discrete_com_pos, const std::array<bool, HexapodConst::LEG_NUM>& is_ground,
		const std::array<DiscreteLegPos, HexapodConst::LEG_NUM>& discretized_leg_pos)
	{
		LegStateBit res = 0;

		res |= static_cast<int>(discrete_com_pos) << kShiftToComNum;	//�d�S�p�^�[���̐��l����bit�𗧂Ă�


		for (int i = 0; i < HexapodConst::LEG_NUM; i++)
		{
			//�ڒn���Ă���Ȃ�Ώ��bit�𗧂Ă�
			if (is_ground[i]) { res[(i + 1) * 4 - 1] = true; }

			// �r��bit�𗧂Ă�
			res |= static_cast<int>(discretized_leg_pos[i]) << (i * 4);
		}

		return res;
	}


	bool IsGrounded(const LegStateBit& leg_state, const int leg_index)
	{
		// leg_index��0�`5�͈̔͂ɂ���K�v������D
		assert(0 <= leg_index );
		assert(leg_index < HexapodConst::LEG_NUM);

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


	LegGroundedBit GetLegGroundedBit(const LegStateBit& leg_state)
	{
		LegGroundedBit res;

		for (int i = 0; i < HexapodConst::LEG_NUM; i++)
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

		return std::move(res);
	}


	int GetGroundedLegNum(const LegStateBit& leg_state)
	{
		int res = 0;

		//�r�̖{�������[�v����
		for (int i = 0; i < HexapodConst::LEG_NUM; i++)
		{
			if (IsGrounded(leg_state, i))
			{
				//�ڒn���Ă���r������΃J�E���g�A�b�v����
				res++;
			}
		}

		return res;
	}


	int GetLiftedLegNum(const LegStateBit& leg_state)
	{
		return HexapodConst::LEG_NUM - GetGroundedLegNum(leg_state);
	}


	void GetGroundedLegIndexByVector(const LegStateBit& leg_state, std::vector<int>* res_index)
	{
		// res_index��nullptr�łȂ����ƁC����ł���K�v������
		assert(res_index != nullptr);
		assert((*res_index).size() == 0);

		//�r��6�{����̂�6�񃋁[�v����
		for (int i = 0; i < HexapodConst::LEG_NUM; i++)
		{
			if (IsGrounded(leg_state, i))
			{
				//�ڒn���Ă���r�̋r�ԍ���vector�ɑ��
				(*res_index).push_back(i);
			}
		}
	}


	void GetLiftedLegIndexByVector(const LegStateBit& leg_state, std::vector<int>* res_index)
	{
		// res_index��nullptr�łȂ����ƁC����ł���K�v������
		assert(res_index != nullptr);
		assert((*res_index).size() == 0);

		//�r��6�{����̂�6�񃋁[�v����
		for (int i = 0; i < HexapodConst::LEG_NUM; i++)
		{
			if (!IsGrounded(leg_state, i))
			{
				//�����Ă���r�̋r�ԍ���vector�ɑ��
				(*res_index).push_back(i);
			}
		}
	}


	DiscreteLegPos GetDiscreteLegPos(const LegStateBit& leg_state, const int leg_index)
	{
		// leg_index��0�`5�͈̔͂ɂ���K�v������D
		assert(0 <= leg_index);
		assert(leg_index < HexapodConst::LEG_NUM);

		const int shift_num = 4 * leg_index;	//4bit�����炷

		const int res = static_cast<int>(((leg_state & (kLegPosMaskbit << shift_num)) >> shift_num).to_ulong());

		return static_cast<DiscreteLegPos>(res);
	}


	DiscreteComPos GetDiscreteComPos(const LegStateBit& leg_state)
	{
		//�d�S�p�^�[����ۑ�����r�b�g���}�X�N���C���̒l�����擾�ł���悤�ɉE�փV�t�g����D
		const int res = static_cast<int>(((leg_state & kComStateMaskbit) >> kShiftToComNum).to_ulong());

		return static_cast<DiscreteComPos>(res);
	}


	bool ChangeLegState(const int leg_index, const DiscreteLegPos new_discretized_leg_pos, const bool is_ground, LegStateBit* leg_state)
	{
		// leg_index��0�`5�͈̔͂ɂ���K�v������D
		assert(0 <= leg_index);
		assert(leg_index < HexapodConst::LEG_NUM);

		// leg_state �� nullptr�ł͂Ȃ�
		assert(leg_state != nullptr);


		//�V�����r��Ԃ𐶐�����
		LegStateBit mask = kLegStateMaskbit << (leg_index * 4);								//4bit�̃f�[�^��ύX����n�_�܂Ń}�X�N�����炷
		LegStateBit  state = static_cast<int>(new_discretized_leg_pos) << (leg_index * 4);	//�r�ʒu�̃f�[�^��4bit�Âz�u����Ă���̂ł��̈ʒu�܂ňړ�����

		//�����Ă���r�̋r�ʒu�݂̂�ύX�i�r���I�_���a�ɂ�����r�b�g�̌��� https://qiita.com/vivisuke/items/bc707190e008551ca07f�j
		LegStateBit res = ((*leg_state) ^ state) & mask;
		(*leg_state) ^= res;

		ChangeGround(leg_index, is_ground, leg_state);

		return true;
	}


	bool ChangeDiscreteLegPos(const int leg_index, const DiscreteLegPos new_discretized_leg_pos, LegStateBit* leg_state)
	{
		// leg_index��0�`5�͈̔͂ɂ���K�v������D
		assert(0 <= leg_index);
		assert(leg_index < HexapodConst::LEG_NUM);

		// leg_state �� nullptr�ł͂Ȃ�
		assert(leg_state != nullptr);


		//�V�����r��Ԃ𐶐�����
		LegStateBit mask = kLegPosMaskbit << (leg_index * 4);								//4bit�̃f�[�^��ύX����n�_�܂Ń}�X�N�����炷
		LegStateBit state = static_cast<int>(new_discretized_leg_pos) << (leg_index * 4);	//�r�ʒu�̃f�[�^��4bit�Âz�u����Ă���̂ł��̈ʒu�܂ňړ�����

		//�����Ă���r�̋r�ʒu�݂̂�ύX�i�r���I�_���a�ɂ�����r�b�g�̌��� https://qiita.com/vivisuke/items/bc707190e008551ca07f�j
		LegStateBit res = ((*leg_state) ^ state) & mask;
		(*leg_state) ^= res;

		return true;
	}


	void ChangeGround(const int leg_index, const bool is_ground, LegStateBit* leg_state)
	{
		// leg_index��0�`5�͈̔͂ɂ���K�v������D
		assert(0 <= leg_index);
		assert(leg_index < HexapodConst::LEG_NUM);

		// leg_state �� nullptr�ł͂Ȃ�
		assert(leg_state != nullptr);


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


	void ChangeAllLegGround(const LegGroundedBit& is_ground_list, LegStateBit* leg_state)
	{
		// leg_state �� nullptr�ł͂Ȃ�
		assert(leg_state != nullptr);

		for (int i = 0; i < HexapodConst::LEG_NUM; i++)
		{
			ChangeGround(i, is_ground_list[i], leg_state);
		}
	}


	void ChangeDiscreteComPos(const DiscreteComPos new_com_pattern, LegStateBit* leg_state)
	{
		// leg_state �� nullptr�ł͂Ȃ�
		assert(leg_state != nullptr);

		const LegStateBit state = static_cast<int>(new_com_pattern) << kShiftToComNum;
		LegStateBit sub = ((*leg_state) ^ state) & kComStateMaskbit;
		(*leg_state) ^= sub;
	}

}	// namespace designlab::leg_func