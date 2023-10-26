#include "pch.h"
//
//#include "../DesignLab/leg_state.h"
//#include "../DesignLab/leg_state.cpp"
//#include "../DesignLab/discrete_leg_pos.h"
//
//
//// SCOPED_TRACE�������Ă����ƁC���s�����Ƃ��ɁC�ǂ̃e�X�g�����s��������\�����Ă����D
//// ���܂肫�ꂢ�Ȏ����ł͂Ȃ����ǁCgoogle test�ɂ��Ẵh�L�������g�����Ȃ��Ă悭�킩���̂ł��̂܂�
//
//
//namespace dl_leg_test
//{
//	class MakeLegState : public ::testing::Test
//	{
//	protected:
//		virtual void SetUp() {}
//		virtual void TearDown() {}
//
//		//! @brief bit�̒l���e�X�e�[�^�X�̒l�Ɠ��������m�F����
//		void testLegState(const std::bitset<dl_leg::kLegStateBitNum>& leg_state, const DiscreteComPos& com_pattern, const bool is_ground[HexapodConst::kLegNum], const DiscreteLegPos leg_pos[HexapodConst::kLegNum])
//		{
//			EXPECT_EQ(dl_leg::GetDiscreteComPos(leg_state), com_pattern) << "bit : " + leg_state.to_string() + "\nfirst : "
//				+ std::to_string(static_cast<int>(dl_leg::GetDiscreteComPos(leg_state))) + " second : " + std::to_string(static_cast<int>(com_pattern));
//
//			for (int i = 0; i < HexapodConst::kLegNum; i++)
//			{
//				EXPECT_EQ(dl_leg::IsGrounded(leg_state, i), is_ground[i]) << "leg index : " + std::to_string(i) + "\nbit : " + leg_state.to_string() + "\nfirst : "
//					+ std::to_string(dl_leg::IsGrounded(leg_state, i)) + " second:" + std::to_string(is_ground[i]);
//				EXPECT_EQ(dl_leg::GetDiscreteLegPos(leg_state, i), leg_pos[i]) << "leg index : " + std::to_string(i) + "\nbit : " + leg_state.to_string() + "\nfirst : "
//					+ std::to_string(dl_leg::GetDiscreteLegPos(leg_state, i)) + " second : " + std::to_string(leg_pos[i]);
//			}
//		}
//	};
//
//
//	TEST_F(MakeLegState, ChangeComPattren)
//	{
//		DiscreteComPos com_pattern = DiscreteComPos::kFront;
//		bool is_ground[HexapodConst::kLegNum] = { true, true, true, true, true, true };
//		DiscreteLegPos discretized_leg_pos[HexapodConst::kLegNum] = { DiscreteLegPos::kCenter, DiscreteLegPos::kCenter, DiscreteLegPos::kCenter,
//																				DiscreteLegPos::kCenter, DiscreteLegPos::kCenter, DiscreteLegPos::kCenter };
//		std::bitset<dl_leg::kLegStateBitNum> res = dl_leg::MakeLegStateBit(com_pattern, is_ground, discretized_leg_pos);
//		{
//			SCOPED_TRACE("first test");
//			testLegState(res, com_pattern, is_ground, discretized_leg_pos);
//		}
//
//
//		//�d�S�p�^�[����ύX���āC�d�S�p�^�[�����ύX���ꂽ���C�܂��d�S�̒l���ς��Ȃ����ƁC�r�ʒu�̒l���ω����Ȃ����Ƃ��m�F
//		com_pattern = DiscreteComPos::kBack;
//		res = dl_leg::MakeLegStateBit(com_pattern, is_ground, discretized_leg_pos);
//		{
//			SCOPED_TRACE("test 1");
//			testLegState(res, com_pattern, is_ground, discretized_leg_pos);
//		}
//
//		com_pattern = DiscreteComPos::kBackLeft;
//		res = dl_leg::MakeLegStateBit(com_pattern, is_ground, discretized_leg_pos);
//		{
//			SCOPED_TRACE("test 2");
//			testLegState(res, com_pattern, is_ground, discretized_leg_pos);
//		}
//
//		com_pattern = DiscreteComPos::kBackRight;
//		res = dl_leg::MakeLegStateBit(com_pattern, is_ground, discretized_leg_pos);
//		{
//			SCOPED_TRACE("test 3");
//			testLegState(res, com_pattern, is_ground, discretized_leg_pos);
//		}
//
//		com_pattern = DiscreteComPos::kFrontLeft;
//		res = dl_leg::MakeLegStateBit(com_pattern, is_ground, discretized_leg_pos);
//		{
//			SCOPED_TRACE("test 4");
//			testLegState(res, com_pattern, is_ground, discretized_leg_pos);
//		}
//
//		com_pattern = DiscreteComPos::kFrontRight;
//		res = dl_leg::MakeLegStateBit(com_pattern, is_ground, discretized_leg_pos);
//		{
//			SCOPED_TRACE("test 5");
//			testLegState(res, com_pattern, is_ground, discretized_leg_pos);
//		}
//
//		com_pattern = DiscreteComPos::kCenterBack;
//		res = dl_leg::MakeLegStateBit(com_pattern, is_ground, discretized_leg_pos);
//		{
//			SCOPED_TRACE("test 6");
//			testLegState(res, com_pattern, is_ground, discretized_leg_pos);
//		}
//
//		com_pattern = DiscreteComPos::kCenterFront;
//		res = dl_leg::MakeLegStateBit(com_pattern, is_ground, discretized_leg_pos);
//		{
//			SCOPED_TRACE("test 7");
//			testLegState(res, com_pattern, is_ground, discretized_leg_pos);
//		}
//	}
//
//	TEST_F(MakeLegState, ChangeGround)
//	{
//		// �l�X�ȋr�ڒn�p�^�[�����쐬���C �r�ڒn�p�^�[�����ύX���ꂽ���C�܂��d�S�̒l���ς��Ȃ����ƁC�r�ʒu�̒l���ω����Ȃ����Ƃ��m�F
//		DiscreteComPos com_pattern = DiscreteComPos::kFront;
//		bool is_ground[HexapodConst::kLegNum] = { true, true, true, true, true, true };
//		DiscreteLegPos discretized_leg_pos[HexapodConst::kLegNum] = { DiscreteLegPos::kCenter, DiscreteLegPos::kCenter, DiscreteLegPos::kCenter,
//																				DiscreteLegPos::kCenter, DiscreteLegPos::kCenter, DiscreteLegPos::kCenter };
//		std::bitset<dl_leg::kLegStateBitNum> res = dl_leg::MakeLegStateBit(com_pattern, is_ground, discretized_leg_pos);
//		{
//			SCOPED_TRACE("first test");
//			testLegState(res, com_pattern, is_ground, discretized_leg_pos);
//		}
//
//
//
//		// 1�r �V�r
//
//		bool is_ground1[HexapodConst::kLegNum] = { false, true, true, true, true, true };
//		res = dl_leg::MakeLegStateBit(com_pattern, is_ground1, discretized_leg_pos);
//		{
//			SCOPED_TRACE("test 1");
//			testLegState(res, com_pattern, is_ground1, discretized_leg_pos);
//		}
//
//
//		bool is_ground2[HexapodConst::kLegNum] = { true, false, true, true, true, true };
//		res = dl_leg::MakeLegStateBit(com_pattern, is_ground2, discretized_leg_pos);
//		{
//			SCOPED_TRACE("test 2");
//			testLegState(res, com_pattern, is_ground2, discretized_leg_pos);
//		}
//
//		bool is_ground3[HexapodConst::kLegNum] = { true, true, false, true, true, true };
//		res = dl_leg::MakeLegStateBit(com_pattern, is_ground3, discretized_leg_pos);
//		{
//			SCOPED_TRACE("test 3");
//			testLegState(res, com_pattern, is_ground3, discretized_leg_pos);
//		}
//
//		bool is_ground4[HexapodConst::kLegNum] = { true, true, true, false, true, true };
//		res = dl_leg::MakeLegStateBit(com_pattern, is_ground4, discretized_leg_pos);
//		{
//			SCOPED_TRACE("test 4");
//			testLegState(res, com_pattern, is_ground4, discretized_leg_pos);
//		}
//
//		bool is_ground5[HexapodConst::kLegNum] = { true, true, true, true, false, true };
//		res = dl_leg::MakeLegStateBit(com_pattern, is_ground5, discretized_leg_pos);
//		{
//			SCOPED_TRACE("test 5");
//			testLegState(res, com_pattern, is_ground5, discretized_leg_pos);
//		}
//
//		bool is_ground6[HexapodConst::kLegNum] = { true, true, true, true, true, false };
//		res = dl_leg::MakeLegStateBit(com_pattern, is_ground6, discretized_leg_pos);
//		{
//			SCOPED_TRACE("test 6");
//			testLegState(res, com_pattern, is_ground6, discretized_leg_pos);
//		}
//
//
//		// 2�r �V�r
//
//		bool is_ground7[HexapodConst::kLegNum] = { false, false, true, true, true, true };
//		res = dl_leg::MakeLegStateBit(com_pattern, is_ground7, discretized_leg_pos);
//		{
//			SCOPED_TRACE("test 7");
//			testLegState(res, com_pattern, is_ground7, discretized_leg_pos);
//		}
//
//		bool is_ground8[HexapodConst::kLegNum] = { true, false, false, true, true, true };
//		res = dl_leg::MakeLegStateBit(com_pattern, is_ground8, discretized_leg_pos);
//		{
//			SCOPED_TRACE("test 8");
//			testLegState(res, com_pattern, is_ground8, discretized_leg_pos);
//		}
//
//		bool is_ground9[HexapodConst::kLegNum] = { true, true, false, false, true, true };
//		res = dl_leg::MakeLegStateBit(com_pattern, is_ground9, discretized_leg_pos);
//		{
//			SCOPED_TRACE("test 9");
//			testLegState(res, com_pattern, is_ground9, discretized_leg_pos);
//		}
//
//		bool is_ground10[HexapodConst::kLegNum] = { true, true, true, false, false, true };
//		res = dl_leg::MakeLegStateBit(com_pattern, is_ground10, discretized_leg_pos);
//		{
//			SCOPED_TRACE("test 10");
//			testLegState(res, com_pattern, is_ground10, discretized_leg_pos);
//		}
//
//		bool is_ground11[HexapodConst::kLegNum] = { true, true, true, true, false, false };
//		res = dl_leg::MakeLegStateBit(com_pattern, is_ground11, discretized_leg_pos);
//		{
//			SCOPED_TRACE("test 11");
//			testLegState(res, com_pattern, is_ground11, discretized_leg_pos);
//		}
//
//		bool is_ground12[HexapodConst::kLegNum] = { false, true, false, true, true, true };
//		res = dl_leg::MakeLegStateBit(com_pattern, is_ground12, discretized_leg_pos);
//		{
//			SCOPED_TRACE("test 12");
//			testLegState(res, com_pattern, is_ground12, discretized_leg_pos);
//		}
//
//		bool is_ground13[HexapodConst::kLegNum] = { true, false, true, false, true, true };
//		res = dl_leg::MakeLegStateBit(com_pattern, is_ground13, discretized_leg_pos);
//		{
//			SCOPED_TRACE("test 13");
//			testLegState(res, com_pattern, is_ground13, discretized_leg_pos);
//		}
//
//		bool is_ground14[HexapodConst::kLegNum] = { true, true, false, true, false, true };
//		res = dl_leg::MakeLegStateBit(com_pattern, is_ground14, discretized_leg_pos);
//		{
//			SCOPED_TRACE("test 14");
//			testLegState(res, com_pattern, is_ground14, discretized_leg_pos);
//		}
//
//		bool is_ground15[HexapodConst::kLegNum] = { true, true, true, false, true, false };
//		res = dl_leg::MakeLegStateBit(com_pattern, is_ground15, discretized_leg_pos);
//		{
//			SCOPED_TRACE("test 15");
//			testLegState(res, com_pattern, is_ground15, discretized_leg_pos);
//		}
//
//		bool is_ground16[HexapodConst::kLegNum] = { false, true, true, false, true, true };
//		res = dl_leg::MakeLegStateBit(com_pattern, is_ground16, discretized_leg_pos);
//		{
//			SCOPED_TRACE("test 16");
//			testLegState(res, com_pattern, is_ground16, discretized_leg_pos);
//		}
//	}
//
//	TEST_F(MakeLegState, ChangeDiscreteLegPos)
//	{
//		//�l�X�ȋr�ʒu�p�^�[�����쐬����D
//		DiscreteComPos com_pattern = DiscreteComPos::kFront;
//		bool is_ground[HexapodConst::kLegNum] = { true, true, true, true, true, true };
//		DiscreteLegPos discretized_leg_pos1[HexapodConst::kLegNum] = { DiscreteLegPos::kCenter, DiscreteLegPos::kCenter, DiscreteLegPos::kCenter,
//																				DiscreteLegPos::kCenter, DiscreteLegPos::kCenter, DiscreteLegPos::kCenter };
//		std::bitset<dl_leg::kLegStateBitNum> res = dl_leg::MakeLegStateBit(com_pattern, is_ground, discretized_leg_pos1);
//		{
//			SCOPED_TRACE("test 1");
//			testLegState(res, com_pattern, is_ground, discretized_leg_pos1);
//		}
//
//
//		// �r�ʒu�p�^�[����ύX���āC �r�ʒu�p�^�[�����ύX���ꂽ���C�܂��d�S�̒l���ς��Ȃ����ƁC�r�ʒu�̒l���ω����Ȃ����Ƃ��m�F
//		DiscreteLegPos discretized_leg_pos2[HexapodConst::kLegNum] = { DiscreteLegPos::kBack, DiscreteLegPos::kCenter, DiscreteLegPos::kLowerBack,
//																				DiscreteLegPos::kCenter, DiscreteLegPos::kUpperFront, DiscreteLegPos::kUpperBack };
//		res = dl_leg::MakeLegStateBit(com_pattern, is_ground, discretized_leg_pos2);
//		{
//			SCOPED_TRACE("test 2");
//			testLegState(res, com_pattern, is_ground, discretized_leg_pos2);
//		}
//
//
//		DiscreteLegPos discretized_leg_pos3[HexapodConst::kLegNum] = { DiscreteLegPos::kLowerBack, DiscreteLegPos::kFront, DiscreteLegPos::kUpperBack,
//																				DiscreteLegPos::kUpperBack, DiscreteLegPos::kLowerBack, DiscreteLegPos::kLowerBack };
//		res = dl_leg::MakeLegStateBit(com_pattern, is_ground, discretized_leg_pos3);
//		{
//			SCOPED_TRACE("test 3");
//			testLegState(res, com_pattern, is_ground, discretized_leg_pos3);
//		}
//
//
//		DiscreteLegPos discretized_leg_pos4[HexapodConst::kLegNum] = { DiscreteLegPos::kUpperBack, DiscreteLegPos::kUpperBack, DiscreteLegPos::kUpperBack,
//																						DiscreteLegPos::kUpperBack, DiscreteLegPos::kUpperBack, DiscreteLegPos::kUpperBack };
//		res = dl_leg::MakeLegStateBit(com_pattern, is_ground, discretized_leg_pos4);
//		{
//			SCOPED_TRACE("test 4");
//			testLegState(res, com_pattern, is_ground, discretized_leg_pos4);
//		}
//	}
//
//}	//namespace dl_leg_test