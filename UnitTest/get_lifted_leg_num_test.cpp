#include "pch.h"

#include "../DesignLab/leg_state.h"


namespace dl_leg_test
{
	TEST(GroundCheck, GetLiftedLegNum)
	{
		// �e�X�g�P�[�X1 �S�r�ڒn
		EDiscreteComPos com = EDiscreteComPos::FRONT;
		bool is_ground[HexapodConst::LEG_NUM] = { true, true, true, true, true, true };
		EDiscreteLegPos discretized_leg_pos[HexapodConst::LEG_NUM] = { EDiscreteLegPos::CENTER, EDiscreteLegPos::CENTER, EDiscreteLegPos::CENTER,
																				EDiscreteLegPos::CENTER, EDiscreteLegPos::CENTER, EDiscreteLegPos::CENTER };
		std::bitset<dl_leg::LEG_STATE_BIT_NUM> res = dl_leg::MakeLegStateBit(com, is_ground, discretized_leg_pos);
		EXPECT_EQ(dl_leg::GetLiftedLegNum(res), 0);


		// �e�X�g�P�[�X2 �ꕔ�r�ڒn ( 3�r )
		bool is_ground2_1[HexapodConst::LEG_NUM] = { true, false, true, false, true, false };
		res = dl_leg::MakeLegStateBit(com, is_ground2_1, discretized_leg_pos);
		EXPECT_EQ(dl_leg::GetLiftedLegNum(res), 3);

		bool is_ground2_2[HexapodConst::LEG_NUM] = { false, true, false, true, false, true };
		res = dl_leg::MakeLegStateBit(com, is_ground2_2, discretized_leg_pos);
		EXPECT_EQ(dl_leg::GetLiftedLegNum(res), 3);

		bool is_ground2_3[HexapodConst::LEG_NUM] = { false, false, true, true, false, true };
		res = dl_leg::MakeLegStateBit(com, is_ground2_3, discretized_leg_pos);
		EXPECT_EQ(dl_leg::GetLiftedLegNum(res), 3);


		// �e�X�g�P�[�X3 �S�r��ڒn
		bool is_ground3[HexapodConst::LEG_NUM] = { false, false, false, false, false, false };
		res = dl_leg::MakeLegStateBit(com, is_ground3, discretized_leg_pos);
		EXPECT_EQ(dl_leg::GetLiftedLegNum(res), 6);


		// �e�X�g�P�[�X4 �ꕔ�r�ڒn ( 4�r )
		bool is_ground4_1[HexapodConst::LEG_NUM] = { false, true, true, true, false, true };
		res = dl_leg::MakeLegStateBit(com, is_ground4_1, discretized_leg_pos);
		EXPECT_EQ(dl_leg::GetLiftedLegNum(res), 2);

		bool is_ground4_2[HexapodConst::LEG_NUM] = { true, false, true, true, true, false };
		res = dl_leg::MakeLegStateBit(com, is_ground4_2, discretized_leg_pos);
		EXPECT_EQ(dl_leg::GetLiftedLegNum(res), 2);

		bool is_ground4_3[HexapodConst::LEG_NUM] = { true, true, false, true, false, true };
		res = dl_leg::MakeLegStateBit(com, is_ground4_3, discretized_leg_pos);
		EXPECT_EQ(dl_leg::GetLiftedLegNum(res), 2);

		bool is_ground4_4[HexapodConst::LEG_NUM] = { true, true, true, false, true, false };
		res = dl_leg::MakeLegStateBit(com, is_ground4_4, discretized_leg_pos);
		EXPECT_EQ(dl_leg::GetLiftedLegNum(res), 2);


		// �e�X�g�P�[�X5 �ꕔ�r�ڒn ( 5�r )
		bool is_ground5_1[HexapodConst::LEG_NUM] = { false, true, true, true, true, true };
		res = dl_leg::MakeLegStateBit(com, is_ground5_1, discretized_leg_pos);
		EXPECT_EQ(dl_leg::GetLiftedLegNum(res), 1);

		bool is_ground5_2[HexapodConst::LEG_NUM] = { true, false, true, true, true, true };
		res = dl_leg::MakeLegStateBit(com, is_ground5_2, discretized_leg_pos);
		EXPECT_EQ(dl_leg::GetLiftedLegNum(res), 1);

		bool is_ground5_3[HexapodConst::LEG_NUM] = { true, true, false, true, true, true };
		res = dl_leg::MakeLegStateBit(com, is_ground5_3, discretized_leg_pos);
		EXPECT_EQ(dl_leg::GetLiftedLegNum(res), 1);

		bool is_ground5_4[HexapodConst::LEG_NUM] = { true, true, true, false, true, true };
		res = dl_leg::MakeLegStateBit(com, is_ground5_4, discretized_leg_pos);
		EXPECT_EQ(dl_leg::GetLiftedLegNum(res), 1);
	}

}
