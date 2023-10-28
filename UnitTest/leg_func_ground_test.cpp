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

	//! @brief �ڒn�E�V�r�̏�Ԃ𕶎���ɕϊ�����
	std::string ToString(const std::array<bool, HexapodConst::kLegNum>& is_ground)
	{
		std::string res = "{ ";
		for (const auto& i : is_ground)
		{
			res += (i ? "�ڒn" : "�V�r");
			res += " ";
		}
		res += "}";
		return res;
	}

	//! @brief �ڒn���Ă���r�̔ԍ��𕶎���ɕϊ�����
	std::string ToString(const std::vector<int>& grounded_leg_index)
	{
		std::string res = "{ ";
		for (const auto& i : grounded_leg_index)
		{
			res += std::to_string(i);
			res += " ";
		}
		res += "}";
		return res;
	}


	// �e�X�g�p�̒萔�D�e�X�g�t�B�N�X�`�����g���ׂ��������̂���...?

	const DiscreteComPos kComPattern = DiscreteComPos::kFront;
	const auto kLegPos = MakeArraySameValue<DiscreteLegPos, HexapodConst::kLegNum>(DiscreteLegPos::kCenter);

	// �ڒn��true, �V�r��false�Ƃ����Ƃ��̋r�̏�Ԃ̔z��ƁC���̂Ƃ��̐ڒn���Ă���r�̔ԍ��̔z��̃^�v��...�̔z��i�΁j
	const std::vector< std::tuple<std::array<bool, HexapodConst::kLegNum>, std::vector<int>> > kTestcaseList{
		{ MakeArraySameValue<bool, HexapodConst::kLegNum>(true),	{ 0, 1, 2, 3, 4, 5 } },
		{ MakeArraySameValue<bool, HexapodConst::kLegNum>(false),	{} },
		{ { true, false, true, false, true, false },	{ 0, 2, 4 } },
		{ { false, true, false, true, false, true },	{ 1, 3, 5 } },
		{ { false, true, true, true, true, true},		{ 1, 2, 3, 4, 5 } },
		{ { true, false, true, true, true, true},		{ 0, 2, 3, 4, 5 } },
		{ { true, true, false, true, true, true},		{ 0, 1, 3, 4, 5 } },
		{ { true, true, true, false, true, true},		{ 0, 1, 2, 4, 5 } },
		{ { true, true, true, true, false, true},		{ 0, 1, 2, 3, 5 } },
		{ { true, true, true, true, true, false},		{ 0, 1, 2, 3, 4 } },
		{ { false, false, true, true, true, true},		{ 2, 3, 4, 5 } },
		{ { false, true, false, true, true, true},		{ 1, 3, 4, 5 } },
		{ { false, true, true, false, true, true},		{ 1, 2, 4, 5 } },
		{ { false, true, true, true, false, true},		{ 1, 2, 3, 5 } },
		{ { false, true, true, true, true, false},		{ 1, 2, 3, 4 } },
		{ { false, false, false, true, true, true},		{ 3, 4, 5 } },
		{ { false, false, true, false, true, true},		{ 2, 4, 5 } },
		{ { false, false, true, true, false, true},		{ 2, 3, 5 } },
		{ { false, false, true, true, true, false},		{ 2, 3, 4 } },
		{ { false, false, false, false, true, true},	{ 4, 5 } },
		{ { false, false, false, true, false, true},	{ 3, 5 } },
		{ { false, false, false, true, true, false},	{ 3, 4 } },
		{ { false, false, false, false, false, true},	{ 5 } },
		{ { false, false, false, false, true, false},	{ 4 } },
	};
}


namespace designlab::test::node::leg_state
{
	// MakeLegStateBit�͐���ɓ��삷�邱�Ƃ�leg_func_test.cpp�Ŋm�F�ς݂Ȃ̂ŁC�����ł͐���ɓ������Ƃ�O��Ƃ��ăe�X�g����

	TEST(LegFuncTest, GetLegGroundedBitTest)
	{
		for (const auto& i : kTestcaseList)
		{
			const auto& is_ground = std::get<0>(i);
			std::ignore = std::get<1>(i);

			dllf::LegStateBit leg_state = dllf::MakeLegStateBit(kComPattern, is_ground, kLegPos);
			dllf::LegGroundedBit leg_ground_bit = dllf::GetLegGroundedBit(leg_state);

			std::string error_message = "���̒l�� " + ToString(is_ground) + " �ł�\n" + 
				"�o�͂��ꂽ�l�� " + leg_ground_bit.to_string() + " �ł�\n";

			for (int j = 0; j < HexapodConst::kLegNum; j++)
			{
				EXPECT_EQ(leg_ground_bit[j], is_ground[j]) << error_message << "index : " << std::to_string(j) << "\n";
			}
		}
	}

	TEST(LegFuncTest, GetGroundedLegNumTest)
	{
		for (const auto& i : kTestcaseList)
		{
			const auto& is_ground = std::get<0>(i);
			const auto& grounded_leg_index = std::get<1>(i);

			dllf::LegStateBit leg_state = dllf::MakeLegStateBit(kComPattern, is_ground, kLegPos);
			int grounded_leg_num = dllf::GetGroundedLegNum(leg_state);

			std::string error_message = "�r�̏�Ԃ�" + ToString(is_ground) + " �ł�\n" +
				"�ڒn�r�̖{���͐�������" + std::to_string(grounded_leg_index.size()) + "�ł�\n" +
				"���ۂɂ�" + std::to_string(grounded_leg_num) + "�ł�";

			EXPECT_EQ(grounded_leg_num, grounded_leg_index.size()) << error_message;
		}
	}

	TEST(LegFuncTest, GetLiftedLegNumTest)
	{
		for (const auto& i : kTestcaseList)
		{
			const auto& is_ground = std::get<0>(i);
			const auto& grounded_leg_index = std::get<1>(i);

			dllf::LegStateBit leg_state = dllf::MakeLegStateBit(kComPattern, is_ground, kLegPos);
			int lifted_leg_num = dllf::GetLiftedLegNum(leg_state);

			std::string error_message = "�r�̏�Ԃ�" + ToString(is_ground) + " �ł�\n" +
				"�V�r�̖{���͐�������" + std::to_string(HexapodConst::kLegNum - grounded_leg_index.size()) + "�ł�\n" +
				"���ۂɂ�" + std::to_string(lifted_leg_num) + "�ł�";

			EXPECT_EQ(lifted_leg_num, HexapodConst::kLegNum - grounded_leg_index.size()) << error_message;
		}
	}

	TEST(LegFuncTest, GetGroundedLegIndexByVectorTest)
	{
		for (const auto& i : kTestcaseList)
		{
			const auto& is_ground = std::get<0>(i);
			const auto& grounded_leg_index = std::get<1>(i);

			dllf::LegStateBit leg_state = dllf::MakeLegStateBit(kComPattern, is_ground, kLegPos);
			std::vector<int> res_index;
			dllf::GetGroundedLegIndexByVector(leg_state, &res_index);

			std::string error_message = "�r�̏�Ԃ�" + ToString(is_ground) + " �ł�\n" +
				"�ڒn���Ă���r�̔ԍ��͐�������" + ToString(grounded_leg_index) + "�ł�\n" +
				"���ۂɂ�" + ToString(res_index) + "�ł�";

			EXPECT_EQ(res_index, grounded_leg_index) << error_message;
		}
	}

	TEST(LegFuncTest, GetLiftedLegIndexByVectorTest)
	{
		for (const auto& i : kTestcaseList)
		{
			const auto& is_ground = std::get<0>(i);
			const auto& grounded_leg_index = std::get<1>(i);

			dllf::LegStateBit leg_state = dllf::MakeLegStateBit(kComPattern, is_ground, kLegPos);
			std::vector<int> res_index;
			dllf::GetLiftedLegIndexByVector(leg_state, &res_index);

			std::vector<int> lifted_leg_index;
			for (int j = 0; j < HexapodConst::kLegNum; j++)
			{
				if (std::find(grounded_leg_index.begin(), grounded_leg_index.end(), j) == grounded_leg_index.end())
				{
					lifted_leg_index.push_back(j);
				}
			}

			std::string error_message = "�r�̏�Ԃ�" + ToString(is_ground) + " �ł�\n" +
				"�V�r���Ă���r�̔ԍ��͐�������" + ToString(lifted_leg_index) + "�ł�\n" +
				"���ۂɂ�" + ToString(res_index) + "�ł�";

			EXPECT_EQ(res_index, lifted_leg_index) << error_message;
		}
	}
}
