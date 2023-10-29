#include "pch.h"

#include "../DesignLab/leg_state.h"


namespace dllf = ::designlab::leg_func;


namespace
{
	//! @brief std::array�𓯂��l�Ŗ��߂�
	template<typename T, size_t S>
	std::array<T, S> MakeArraySameValue(const T& value)
	{
		std::array<T, S> res;
		res.fill(value);
		return res;
	}

	const DiscreteComPos first_com_pattern_ = DiscreteComPos::kFront;
	const std::array<bool, HexapodConst::kLegNum> first_is_ground_ = MakeArraySameValue<bool, HexapodConst::kLegNum>(true);
	const std::array<DiscreteLegPos, HexapodConst::kLegNum> first_leg_pos_ = MakeArraySameValue<DiscreteLegPos, HexapodConst::kLegNum>(DiscreteLegPos::kCenter);

	const dllf::LegStateBit first_leg_state_ = dllf::MakeLegStateBit(first_com_pattern_, first_is_ground_, first_leg_pos_);
}


namespace designlab::test::node::leg_state
{
	TEST(LegFuncTest, ChangeLegStateTest)
	{
		// �ύX����index�C�ύX��̋r�̏�ԁC�ύX��̐ڒn��ԁC�̏��̃^�v��
		const std::vector<std::tuple<int, DiscreteLegPos, bool>> testcase_list{
			{ 0, DiscreteLegPos::kFront, false},
			{ 1, DiscreteLegPos::kBack , false},
			{ 2, DiscreteLegPos::kCenter, false},
			{ 3, DiscreteLegPos::kLowerBack, false},
			{ 4, DiscreteLegPos::kLowerFront, false},
			{ 5, DiscreteLegPos::kUpperBack, false},
			{ 0, DiscreteLegPos::kFront, true},
			{ 1, DiscreteLegPos::kBack , true},
			{ 2, DiscreteLegPos::kCenter, true},
			{ 3, DiscreteLegPos::kLowerBack, true},
			{ 4, DiscreteLegPos::kLowerFront, true},
			{ 5, DiscreteLegPos::kUpperBack, true},
		};

		for (const auto& i : testcase_list)
		{
			dllf::LegStateBit leg_state = first_leg_state_;

			//tuple����l�����o��
			int index = std::get<0>(i);
			DiscreteLegPos leg_pos = std::get<1>(i);
			bool is_ground = std::get<2>(i);

			//�r�̏�Ԃ�ύX����
			dllf::ChangeLegState(index, leg_pos, is_ground, &leg_state);

			//�ύX�����r�̈ʒu�����������m�F����D�璷�����C�_�u���`�F�b�N����D
			for (int j = 0; j < HexapodConst::kLegNum; j++)
			{
				if (j == index)
				{
					EXPECT_EQ(dllf::GetDiscreteLegPos(leg_state, j), leg_pos) << "���U�����ꂽ�r�ʒu�͎w�肳�ꂽ�l�ɕύX�����K�v������܂��D";
					EXPECT_EQ(dllf::IsGrounded(leg_state, j), is_ground) << "�w�肳�ꂽ�l�ɕύX�����K�v������܂��D";
				}
				else
				{
					EXPECT_EQ(dllf::GetDiscreteLegPos(leg_state, j), first_leg_pos_[j]) << "�w�肳��Ă��Ȃ��l�͕ύX����Ă͂����܂���D";
					EXPECT_EQ(dllf::GetDiscreteLegPos(leg_state, j), dllf::GetDiscreteLegPos(first_leg_state_, j)) << "�w�肳��Ă��Ȃ��l�͕ύX����Ă͂����܂���D";
					EXPECT_EQ(dllf::IsGrounded(leg_state, j), first_is_ground_[j]) << "�w�肳��Ă��Ȃ��l�͕ύX����Ă͂����܂���D";
					EXPECT_EQ(dllf::IsGrounded(leg_state, j), dllf::IsGrounded(first_leg_state_, j)) << "�w�肳��Ă��Ȃ��l�͕ύX����Ă͂����܂���D";
				}
			}

			EXPECT_EQ(dllf::GetDiscreteComPos(leg_state), first_com_pattern_) << "�d�S�p�^�[���͕ύX����Ă͂����܂���D";
			EXPECT_EQ(dllf::GetDiscreteComPos(leg_state), dllf::GetDiscreteComPos(first_leg_state_)) << "�d�S�p�^�[���͕ύX����Ă͂����܂���D";
		}
	}

	TEST(LegFuncTest, ChangeDiscreteLegPosTest)
	{
		// �ύX����index�C�ύX��̋r�̏�Ԃ̏��̃^�v���D
		const std::vector<std::tuple<int, DiscreteLegPos>> testcase_list{
			{ 0, DiscreteLegPos::kFront},
			{ 1, DiscreteLegPos::kBack },
			{ 2, DiscreteLegPos::kCenter},
			{ 3, DiscreteLegPos::kLowerBack},
			{ 4, DiscreteLegPos::kLowerFront},
			{ 5, DiscreteLegPos::kUpperBack},
			{ 2, DiscreteLegPos::kFront},
			{ 3, DiscreteLegPos::kBack },
			{ 4, DiscreteLegPos::kCenter},
			{ 5, DiscreteLegPos::kLowerBack},
			{ 0, DiscreteLegPos::kLowerFront},
			{ 1, DiscreteLegPos::kUpperBack},
		};

		for (const auto& i : testcase_list)
		{
			dllf::LegStateBit leg_state = first_leg_state_;

			//tuple����l�����o���D
			int index = std::get<0>(i);
			DiscreteLegPos leg_pos = std::get<1>(i);

			//�r�̏�Ԃ�ύX����D
			dllf::ChangeDiscreteLegPos(index, leg_pos, &leg_state);

			//�ύX�����r�̈ʒu�����������m�F����D�璷�����C�_�u���`�F�b�N����D
			for (int j = 0; j < HexapodConst::kLegNum; j++)
			{
				if (j == index)
				{
					EXPECT_EQ(dllf::GetDiscreteLegPos(leg_state, j), leg_pos) << "���U�����ꂽ�r�ʒu�͎w�肳�ꂽ�l�ɕύX�����K�v������܂��D";
				}
				else
				{
					EXPECT_EQ(dllf::GetDiscreteLegPos(leg_state, j), first_leg_pos_[j]) << "�w�肳��Ă��Ȃ��l�͕ύX����Ă͂����܂���D";
					EXPECT_EQ(dllf::GetDiscreteLegPos(leg_state, j), dllf::GetDiscreteLegPos(first_leg_state_, j)) << "�w�肳��Ă��Ȃ��l�͕ύX����Ă͂����܂���D";
				}

				EXPECT_EQ(dllf::IsGrounded(leg_state, j), first_is_ground_[j]) << "�ڒn��Ԃ͕ύX����Ă͂����܂���D";
				EXPECT_EQ(dllf::IsGrounded(leg_state, j), dllf::IsGrounded(first_leg_state_, j)) << "�ڒn��Ԃ͕ύX����Ă͂����܂���D";
			}

			EXPECT_EQ(dllf::GetDiscreteComPos(leg_state), first_com_pattern_) << "�d�S�p�^�[���͕ύX����Ă͂����܂���D";
			EXPECT_EQ(dllf::GetDiscreteComPos(leg_state), dllf::GetDiscreteComPos(first_leg_state_)) << "�d�S�p�^�[���͕ύX����Ă͂����܂���D";
		}
	}

	TEST(LegFuncTest, ChangeIsGroundTest)
	{
		// �ύX����index�C�ύX��̐ڒn��Ԃ̏��̃^�v��
		const std::vector<std::tuple<int, bool>> testcase_list{
			{ 0, false},
			{ 1, false},
			{ 2, false},
			{ 3, false},
			{ 4, false},
			{ 5, false},
			{ 0, true},
			{ 1, true},
			{ 2, true},
			{ 3, true},
			{ 4, true},
			{ 5, true},
		};

		for (const auto& i : testcase_list)
		{
			dllf::LegStateBit leg_state = first_leg_state_;

			//tuple����l�����o��
			int index = std::get<0>(i);
			bool is_ground = std::get<1>(i);

			//�r�̏�Ԃ�ύX����
			dllf::ChangeGround(index, is_ground, &leg_state);

			//�ύX�����r�̈ʒu�����������m�F����D�璷�����C�_�u���`�F�b�N����D
			for (int j = 0; j < HexapodConst::kLegNum; j++)
			{
				if (j == index)
				{
					EXPECT_EQ(dllf::IsGrounded(leg_state, j), is_ground) << "�w�肳�ꂽ�l�ɕύX�����K�v������܂��D";
				}
				else
				{
					EXPECT_EQ(dllf::IsGrounded(leg_state, j), first_is_ground_[j]) << "�w�肳��Ă��Ȃ��l�͕ύX����Ă͂����܂���D";
					EXPECT_EQ(dllf::IsGrounded(leg_state, j), dllf::IsGrounded(first_leg_state_, j)) << "�w�肳��Ă��Ȃ��l�͕ύX����Ă͂����܂���D";
				}

				EXPECT_EQ(dllf::GetDiscreteLegPos(leg_state, j), first_leg_pos_[j]) << "�w�肳��Ă��Ȃ��l�͕ύX����Ă͂����܂���D";
				EXPECT_EQ(dllf::GetDiscreteLegPos(leg_state, j), dllf::GetDiscreteLegPos(first_leg_state_, j)) << "�w�肳��Ă��Ȃ��l�͕ύX����Ă͂����܂���D";
			}

			EXPECT_EQ(dllf::GetDiscreteComPos(leg_state), first_com_pattern_) << "�d�S�p�^�[���͕ύX����Ă͂����܂���D";
			EXPECT_EQ(dllf::GetDiscreteComPos(leg_state), dllf::GetDiscreteComPos(first_leg_state_)) << "�d�S�p�^�[���͕ύX����Ă͂����܂���D";
		}
	}

	TEST(LegFuncTest, ChangeDiscreteComPosTest)
	{
		// �ύX����index�C�ύX��̏d�S�p�^�[���̏��̃^�v��
		const std::vector<DiscreteComPos> testcase_list{
			{ DiscreteComPos::kFront},
			{ DiscreteComPos::kBack },
			{ DiscreteComPos::kBackLeft},
			{ DiscreteComPos::kBackRight},
			{ DiscreteComPos::kCenterBack},
			{ DiscreteComPos::kCenterFront},
			{ DiscreteComPos::kFrontLeft},
			{ DiscreteComPos::kFrontRight},
		};

		for (const auto& i : testcase_list)
		{
			dllf::LegStateBit leg_state = first_leg_state_;

			//tuple����l�����o��
			DiscreteComPos com_pos = i;

			//�r�̏�Ԃ�ύX����
			dllf::ChangeDiscreteComPos(com_pos, &leg_state);

			//�ύX�����r�̈ʒu�����������m�F����
			for (int j = 0; j < HexapodConst::kLegNum; j++)
			{
				// �璷�����C�_�u���`�F�b�N

				EXPECT_EQ(dllf::IsGrounded(leg_state, j), first_is_ground_[j]) << "�ڒn��Ԃ͕ύX����Ă͂����܂���D";
				EXPECT_EQ(dllf::IsGrounded(leg_state, j), dllf::IsGrounded(first_leg_state_, j)) << "�ڒn��Ԃ͕ύX����Ă͂����܂���D";
				EXPECT_EQ(dllf::GetDiscreteLegPos(leg_state, j), first_leg_pos_[j]) << "�w�肳��Ă��Ȃ��l�͕ύX����Ă͂����܂���D";
				EXPECT_EQ(dllf::GetDiscreteLegPos(leg_state, j), dllf::GetDiscreteLegPos(first_leg_state_, j)) << "�w�肳��Ă��Ȃ��l�͕ύX����Ă͂����܂���D";
			}

			EXPECT_EQ(dllf::GetDiscreteComPos(leg_state), com_pos) << "�w�肳�ꂽ�l�ɕύX�����K�v������܂��D";
		}
	}

	TEST(LegFuncTest, ChangeAllLegGround)
	{
		std::vector<dllf::LegGroundedBit> testcase_list{
			0b111111,	0b011111,	0b101111,	0b110111,	0b111011,	0b111101,
			0b111110,	0b011110,	0b101110,	0b110110,	0b111010,	0b111100,
			0b111100,	0b011100,	0b101100,	0b110100,	0b111000,	0b111010,
		};

		for (const auto& i : testcase_list) 
		{
			dllf::LegStateBit leg_state = first_leg_state_;

			//�r�̏�Ԃ�ύX����
			dllf::ChangeAllLegGround(i, &leg_state);

			//�ύX�����r�̈ʒu�����������m�F����
			for (int j = 0; j < HexapodConst::kLegNum; j++)
			{
				// �璷�����C�_�u���`�F�b�N

				EXPECT_EQ(dllf::IsGrounded(leg_state, j), i[j]) << "�w�肳�ꂽ�l�ɕύX�����K�v������܂��D";
				EXPECT_EQ(dllf::GetDiscreteLegPos(leg_state, j), first_leg_pos_[j]) << "�w�肳��Ă��Ȃ��l�͕ύX����Ă͂����܂���D";
				EXPECT_EQ(dllf::GetDiscreteLegPos(leg_state, j), dllf::GetDiscreteLegPos(first_leg_state_, j)) << "�w�肳��Ă��Ȃ��l�͕ύX����Ă͂����܂���D";
			}

			EXPECT_EQ(dllf::GetDiscreteComPos(leg_state), first_com_pattern_) << "�d�S�p�^�[���͕ύX����Ă͂����܂���D";
			EXPECT_EQ(dllf::GetDiscreteComPos(leg_state), dllf::GetDiscreteComPos(first_leg_state_)) << "�d�S�p�^�[���͕ύX����Ă͂����܂���D";
		}
	}

}	// namespace designlab::test::node::leg_state