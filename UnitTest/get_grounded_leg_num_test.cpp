#include "pch.h"
//
//#include "../DesignLab/leg_state.h"
//
//
//namespace dl_leg_test
//{
//	TEST(GroundCheck, GetGroundedLegNum)
//	{
//		// �e�X�g�P�[�X1 �S�r�ڒn
//		DiscreteComPos com = DiscreteComPos::kFront;
//		bool is_ground[HexapodConst::kLegNum] = { true, true, true, true, true, true };
//		DiscreteLegPos discretized_leg_pos[HexapodConst::kLegNum] = { DiscreteLegPos::kCenter, DiscreteLegPos::kCenter, DiscreteLegPos::kCenter,
//																				DiscreteLegPos::kCenter, DiscreteLegPos::kCenter, DiscreteLegPos::kCenter };
//		std::bitset<dl_leg::kLegStateBitNum> res = dl_leg::MakeLegStateBit(com, is_ground, discretized_leg_pos);
//		EXPECT_EQ(dl_leg::GetGroundedLegNum(res), 6);
//
//
//		// �e�X�g�P�[�X2 �ꕔ�r�ڒn ( 3�r )
//		bool is_ground2_1[HexapodConst::kLegNum] = { true, false, true, false, true, false };
//		res = dl_leg::MakeLegStateBit(com, is_ground2_1, discretized_leg_pos);
//		EXPECT_EQ(dl_leg::GetGroundedLegNum(res), 3);
//
//		bool is_ground2_2[HexapodConst::kLegNum] = { false, true, false, true, false, true };
//		res = dl_leg::MakeLegStateBit(com, is_ground2_2, discretized_leg_pos);
//		EXPECT_EQ(dl_leg::GetGroundedLegNum(res), 3);
//
//		bool is_ground2_3[HexapodConst::kLegNum] = { false, false, true, true, false, true };
//		res = dl_leg::MakeLegStateBit(com, is_ground2_3, discretized_leg_pos);
//		EXPECT_EQ(dl_leg::GetGroundedLegNum(res), 3);
//
//
//		// �e�X�g�P�[�X3 �S�r��ڒn
//		bool is_ground3[HexapodConst::kLegNum] = { false, false, false, false, false, false };
//		res = dl_leg::MakeLegStateBit(com, is_ground3, discretized_leg_pos);
//		EXPECT_EQ(dl_leg::GetGroundedLegNum(res), 0);
//
//
//		// �e�X�g�P�[�X4 �ꕔ�r�ڒn ( 4�r )
//		bool is_ground4_1[HexapodConst::kLegNum] = { false, true, true, true, false, true };
//		res = dl_leg::MakeLegStateBit(com, is_ground4_1, discretized_leg_pos);
//		EXPECT_EQ(dl_leg::GetGroundedLegNum(res), 4);
//
//		bool is_ground4_2[HexapodConst::kLegNum] = { true, false, true, true, true, false };
//		res = dl_leg::MakeLegStateBit(com, is_ground4_2, discretized_leg_pos);
//		EXPECT_EQ(dl_leg::GetGroundedLegNum(res), 4);
//
//		bool is_ground4_3[HexapodConst::kLegNum] = { true, true, false, true, false, true };
//		res = dl_leg::MakeLegStateBit(com, is_ground4_3, discretized_leg_pos);
//		EXPECT_EQ(dl_leg::GetGroundedLegNum(res), 4);
//
//		bool is_ground4_4[HexapodConst::kLegNum] = { true, true, true, false, true, false };
//		res = dl_leg::MakeLegStateBit(com, is_ground4_4, discretized_leg_pos);
//		EXPECT_EQ(dl_leg::GetGroundedLegNum(res), 4);
//
//
//		// �e�X�g�P�[�X5 �ꕔ�r�ڒn ( 5�r )
//		bool is_ground5_1[HexapodConst::kLegNum] = { false, true, true, true, true, true };
//		res = dl_leg::MakeLegStateBit(com, is_ground5_1, discretized_leg_pos);
//		EXPECT_EQ(dl_leg::GetGroundedLegNum(res), 5);
//
//		bool is_ground5_2[HexapodConst::kLegNum] = { true, false, true, true, true, true };
//		res = dl_leg::MakeLegStateBit(com, is_ground5_2, discretized_leg_pos);
//		EXPECT_EQ(dl_leg::GetGroundedLegNum(res), 5);
//
//		bool is_ground5_3[HexapodConst::kLegNum] = { true, true, false, true, true, true };
//		res = dl_leg::MakeLegStateBit(com, is_ground5_3, discretized_leg_pos);
//		EXPECT_EQ(dl_leg::GetGroundedLegNum(res), 5);
//
//		bool is_ground5_4[HexapodConst::kLegNum] = { true, true, true, false, true, true };
//		res = dl_leg::MakeLegStateBit(com, is_ground5_4, discretized_leg_pos);
//		EXPECT_EQ(dl_leg::GetGroundedLegNum(res), 5);
//	}
//}