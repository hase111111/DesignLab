#include "pch.h"
//
//#include "../DesignLab/leg_state.h"
//
//
//namespace dl_leg_test
//{
//	class GroundCheckWithVector : public ::testing::Test
//	{
//	protected:
//		void SetUp() override {}
//		void TearDown() override {}
//
//		//! @brief bool�^�̔z���vector��index����v���邩�m�F����
//		void testBoolArrayAndVectorIndex(const bool bool_array[HexapodConst::kLegNum], const std::vector<int>& vec)
//		{
//			int cnt = 0;
//
//			for (int i = 0; i < HexapodConst::kLegNum; i++)
//			{
//				if (bool_array[i]) { cnt++; }
//			}
//
//			ASSERT_EQ(cnt, vec.size());
//
//			for (int i = 0; i < vec.size(); i++)
//			{
//				ASSERT_EQ(bool_array[vec[i]], true);
//			}
//		}
//	};
//
//
//	TEST_F(GroundCheckWithVector, GetGroundedLegIndexWithVector)
//	{
//		// �e�X�g�P�[�X1 �S�r�ڒn
//		DiscreteComPos com = DiscreteComPos::kFront;
//		bool is_ground[HexapodConst::kLegNum] = { true, true, true, true, true, true };
//		DiscreteLegPos discretized_leg_pos[HexapodConst::kLegNum] = { DiscreteLegPos::kCenter, DiscreteLegPos::kCenter, DiscreteLegPos::kCenter,
//																				DiscreteLegPos::kCenter, DiscreteLegPos::kCenter, DiscreteLegPos::kCenter };
//		std::bitset<dllf::kLegStateBitNum> res = dllf::MakeLegStateBit(com, is_ground, discretized_leg_pos);
//		std::vector<int> grounded_leg_index;
//		dllf::GetGroundedLegIndexByVector(res, &grounded_leg_index);
//		testBoolArrayAndVectorIndex(is_ground, grounded_leg_index);
//
//
//		// �e�X�g�P�[�X2 5�r�ڒn
//		bool is_ground1_1[HexapodConst::kLegNum] = { true, true, true, true, true, false };
//		res = dllf::MakeLegStateBit(com, is_ground1_1, discretized_leg_pos);
//		dllf::GetGroundedLegIndexByVector(res, &grounded_leg_index);
//		testBoolArrayAndVectorIndex(is_ground1_1, grounded_leg_index);
//
//		bool is_ground1_2[HexapodConst::kLegNum] = { true, true, true, true, false, true };
//		res = dllf::MakeLegStateBit(com, is_ground1_2, discretized_leg_pos);
//		dllf::GetGroundedLegIndexByVector(res, &grounded_leg_index);
//		testBoolArrayAndVectorIndex(is_ground1_2, grounded_leg_index);
//	}
//}