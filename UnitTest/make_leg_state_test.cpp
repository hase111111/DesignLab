#include "pch.h"

#include "../DesignLab/leg_state.h"
#include "../DesignLab/leg_state.cpp"


namespace dl_leg_test
{
	class MakeLegState : public ::testing::Test
	{
	protected:
		virtual void SetUp() {}
		virtual void TearDown() {}

		//! @brief bit�̒l���e�X�e�[�^�X�̒l�Ɠ��������m�F����
		void testLegState(const std::bitset<dl_leg::LEG_STATE_BIT_NUM>& leg_state, const dl_com::EComPattern& com_pattern, const bool is_ground[HexapodConst::LEG_NUM], const dl_leg::EDiscreteLegPos leg_pos[HexapodConst::LEG_NUM])
		{
			EXPECT_EQ(dl_leg::getComPatternState(leg_state), com_pattern) << "bit:" + leg_state.to_string() + "\n first:"
				+ std::to_string(static_cast<int>(dl_leg::getComPatternState(leg_state))) + " second:" + std::to_string(static_cast<int>(com_pattern));

			for (int i = 0; i < HexapodConst::LEG_NUM; i++)
			{
				EXPECT_EQ(dl_leg::isGrounded(leg_state, i), is_ground[i]) << std::to_string(i) + " bit:" + leg_state.to_string() + "\n first:"
					+ std::to_string(dl_leg::isGrounded(leg_state, i)) + " second:" + std::to_string(is_ground[i]);
				EXPECT_EQ(dl_leg::getLegState(leg_state, i), leg_pos[i]) << std::to_string(i) + " bit:" + leg_state.to_string() + "\n first:"
					+ std::to_string(dl_leg::getLegState(leg_state, i)) + " second:" + std::to_string(leg_pos[i]);
			}
		}
	};


	TEST_F(MakeLegState, MakeLegStateChangeComPattren)
	{
		dl_com::EComPattern com_pattern = dl_com::EComPattern::FRONT;
		bool is_ground[HexapodConst::LEG_NUM] = { true, true, true, true, true, true };
		dl_leg::EDiscreteLegPos discretized_leg_pos[HexapodConst::LEG_NUM] = { dl_leg::EDiscreteLegPos::CENTER, dl_leg::EDiscreteLegPos::CENTER, dl_leg::EDiscreteLegPos::CENTER,
																				dl_leg::EDiscreteLegPos::CENTER, dl_leg::EDiscreteLegPos::CENTER, dl_leg::EDiscreteLegPos::CENTER };
		std::bitset<dl_leg::LEG_STATE_BIT_NUM> res = dl_leg::makeLegState(com_pattern, is_ground, discretized_leg_pos);

		testLegState(res, com_pattern, is_ground, discretized_leg_pos);


		//�d�S�p�^�[����ύX���āC�d�S�p�^�[�����ύX���ꂽ���C�܂��d�S�̒l���ς��Ȃ����ƁC�r�ʒu�̒l���ω����Ȃ����Ƃ��m�F
		com_pattern = dl_com::EComPattern::BACK;
		res = dl_leg::makeLegState(com_pattern, is_ground, discretized_leg_pos);
		testLegState(res, com_pattern, is_ground, discretized_leg_pos);

		com_pattern = dl_com::EComPattern::BACK_LEFT;
		res = dl_leg::makeLegState(com_pattern, is_ground, discretized_leg_pos);
		testLegState(res, com_pattern, is_ground, discretized_leg_pos);

		com_pattern = dl_com::EComPattern::BACK_RIGHT;
		res = dl_leg::makeLegState(com_pattern, is_ground, discretized_leg_pos);
		testLegState(res, com_pattern, is_ground, discretized_leg_pos);

		com_pattern = dl_com::EComPattern::FRONT_LEFT;
		res = dl_leg::makeLegState(com_pattern, is_ground, discretized_leg_pos);
		testLegState(res, com_pattern, is_ground, discretized_leg_pos);

		com_pattern = dl_com::EComPattern::FRONT_RIGHT;
		res = dl_leg::makeLegState(com_pattern, is_ground, discretized_leg_pos);
		testLegState(res, com_pattern, is_ground, discretized_leg_pos);

		com_pattern = dl_com::EComPattern::CENTER_BACK;
		res = dl_leg::makeLegState(com_pattern, is_ground, discretized_leg_pos);
		testLegState(res, com_pattern, is_ground, discretized_leg_pos);

		com_pattern = dl_com::EComPattern::CENTER_FRONT;
		res = dl_leg::makeLegState(com_pattern, is_ground, discretized_leg_pos);
		testLegState(res, com_pattern, is_ground, discretized_leg_pos);
	}

	TEST_F(MakeLegState, MakeLegStateChangeGround)
	{
		// �l�X�ȋr�ڒn�p�^�[�����쐬���C �r�ڒn�p�^�[�����ύX���ꂽ���C�܂��d�S�̒l���ς��Ȃ����ƁC�r�ʒu�̒l���ω����Ȃ����Ƃ��m�F
		dl_com::EComPattern com_pattern = dl_com::EComPattern::FRONT;
		bool is_ground[HexapodConst::LEG_NUM] = { true, true, true, true, true, true };
		dl_leg::EDiscreteLegPos discretized_leg_pos[HexapodConst::LEG_NUM] = { dl_leg::EDiscreteLegPos::CENTER, dl_leg::EDiscreteLegPos::CENTER, dl_leg::EDiscreteLegPos::CENTER,
																				dl_leg::EDiscreteLegPos::CENTER, dl_leg::EDiscreteLegPos::CENTER, dl_leg::EDiscreteLegPos::CENTER };
		std::bitset<dl_leg::LEG_STATE_BIT_NUM> res = dl_leg::makeLegState(com_pattern, is_ground, discretized_leg_pos);
		testLegState(res, com_pattern, is_ground, discretized_leg_pos);


		// 1�r �V�r

		bool is_ground1[HexapodConst::LEG_NUM] = { false, true, true, true, true, true };
		res = dl_leg::makeLegState(com_pattern, is_ground1, discretized_leg_pos);
		testLegState(res, com_pattern, is_ground1, discretized_leg_pos);

		bool is_ground2[HexapodConst::LEG_NUM] = { true, false, true, true, true, true };
		res = dl_leg::makeLegState(com_pattern, is_ground2, discretized_leg_pos);
		testLegState(res, com_pattern, is_ground2, discretized_leg_pos);

		bool is_ground3[HexapodConst::LEG_NUM] = { true, true, false, true, true, true };
		res = dl_leg::makeLegState(com_pattern, is_ground3, discretized_leg_pos);
		testLegState(res, com_pattern, is_ground3, discretized_leg_pos);

		bool is_ground4[HexapodConst::LEG_NUM] = { true, true, true, false, true, true };
		res = dl_leg::makeLegState(com_pattern, is_ground4, discretized_leg_pos);
		testLegState(res, com_pattern, is_ground4, discretized_leg_pos);

		bool is_ground5[HexapodConst::LEG_NUM] = { true, true, true, true, false, true };
		res = dl_leg::makeLegState(com_pattern, is_ground5, discretized_leg_pos);
		testLegState(res, com_pattern, is_ground5, discretized_leg_pos);

		bool is_ground6[HexapodConst::LEG_NUM] = { true, true, true, true, true, false };
		res = dl_leg::makeLegState(com_pattern, is_ground6, discretized_leg_pos);
		testLegState(res, com_pattern, is_ground6, discretized_leg_pos);


		// 2�r �V�r

		bool is_ground7[HexapodConst::LEG_NUM] = { false, false, true, true, true, true };
		res = dl_leg::makeLegState(com_pattern, is_ground7, discretized_leg_pos);
		testLegState(res, com_pattern, is_ground7, discretized_leg_pos);

		bool is_ground8[HexapodConst::LEG_NUM] = { true, false, false, true, true, true };
		res = dl_leg::makeLegState(com_pattern, is_ground8, discretized_leg_pos);
		testLegState(res, com_pattern, is_ground8, discretized_leg_pos);

		bool is_ground9[HexapodConst::LEG_NUM] = { true, true, false, false, true, true };
		res = dl_leg::makeLegState(com_pattern, is_ground9, discretized_leg_pos);
		testLegState(res, com_pattern, is_ground9, discretized_leg_pos);

		bool is_ground10[HexapodConst::LEG_NUM] = { true, true, true, false, false, true };
		res = dl_leg::makeLegState(com_pattern, is_ground10, discretized_leg_pos);
		testLegState(res, com_pattern, is_ground10, discretized_leg_pos);

		bool is_ground11[HexapodConst::LEG_NUM] = { true, true, true, true, false, false };
		res = dl_leg::makeLegState(com_pattern, is_ground11, discretized_leg_pos);
		testLegState(res, com_pattern, is_ground11, discretized_leg_pos);

		bool is_ground12[HexapodConst::LEG_NUM] = { false, true, false, true, true, true };
		res = dl_leg::makeLegState(com_pattern, is_ground12, discretized_leg_pos);
		testLegState(res, com_pattern, is_ground12, discretized_leg_pos);

		bool is_ground13[HexapodConst::LEG_NUM] = { true, false, true, false, true, true };
		res = dl_leg::makeLegState(com_pattern, is_ground13, discretized_leg_pos);
		testLegState(res, com_pattern, is_ground13, discretized_leg_pos);

		bool is_ground14[HexapodConst::LEG_NUM] = { true, true, false, true, false, true };
		res = dl_leg::makeLegState(com_pattern, is_ground14, discretized_leg_pos);
		testLegState(res, com_pattern, is_ground14, discretized_leg_pos);

		bool is_ground15[HexapodConst::LEG_NUM] = { true, true, true, false, true, false };
		res = dl_leg::makeLegState(com_pattern, is_ground15, discretized_leg_pos);
		testLegState(res, com_pattern, is_ground15, discretized_leg_pos);

		bool is_ground16[HexapodConst::LEG_NUM] = { false, true, true, false, true, true };
		res = dl_leg::makeLegState(com_pattern, is_ground16, discretized_leg_pos);
		testLegState(res, com_pattern, is_ground16, discretized_leg_pos);
	}

	TEST_F(MakeLegState, MAkeLegStateChangeDiscreteLegPos)
	{
		//�l�X�ȋr�ʒu�p�^�[�����쐬����D
		dl_com::EComPattern com_pattern = dl_com::EComPattern::FRONT;
		bool is_ground[HexapodConst::LEG_NUM] = { true, true, true, true, true, true };
		dl_leg::EDiscreteLegPos discretized_leg_pos1[HexapodConst::LEG_NUM] = { dl_leg::EDiscreteLegPos::CENTER, dl_leg::EDiscreteLegPos::CENTER, dl_leg::EDiscreteLegPos::CENTER,
																				dl_leg::EDiscreteLegPos::CENTER, dl_leg::EDiscreteLegPos::CENTER, dl_leg::EDiscreteLegPos::CENTER };
		std::bitset<dl_leg::LEG_STATE_BIT_NUM> res = dl_leg::makeLegState(com_pattern, is_ground, discretized_leg_pos1);
		testLegState(res, com_pattern, is_ground, discretized_leg_pos1);


		// �r�ʒu�p�^�[����ύX���āC �r�ʒu�p�^�[�����ύX���ꂽ���C�܂��d�S�̒l���ς��Ȃ����ƁC�r�ʒu�̒l���ω����Ȃ����Ƃ��m�F
		dl_leg::EDiscreteLegPos discretized_leg_pos2[HexapodConst::LEG_NUM] = { dl_leg::EDiscreteLegPos::BACK, dl_leg::EDiscreteLegPos::CENTER, dl_leg::EDiscreteLegPos::LOWER_BACK,
																				dl_leg::EDiscreteLegPos::CENTER, dl_leg::EDiscreteLegPos::UPPER_FRONT, dl_leg::EDiscreteLegPos::UPPER_BACK };
		res = dl_leg::makeLegState(com_pattern, is_ground, discretized_leg_pos2);
		testLegState(res, com_pattern, is_ground, discretized_leg_pos2);


		dl_leg::EDiscreteLegPos discretized_leg_pos3[HexapodConst::LEG_NUM] = { dl_leg::EDiscreteLegPos::LOWER_BACK, dl_leg::EDiscreteLegPos::FRONT, dl_leg::EDiscreteLegPos::UPPER_BACK,
																				dl_leg::EDiscreteLegPos::UPPER_BACK, dl_leg::EDiscreteLegPos::LOWER_BACK, dl_leg::EDiscreteLegPos::LOWER_BACK };
		res = dl_leg::makeLegState(com_pattern, is_ground, discretized_leg_pos3);
		testLegState(res, com_pattern, is_ground, discretized_leg_pos3);


		dl_leg::EDiscreteLegPos discretized_leg_pos4[HexapodConst::LEG_NUM] = { dl_leg::EDiscreteLegPos::UPPER_BACK, dl_leg::EDiscreteLegPos::UPPER_BACK, dl_leg::EDiscreteLegPos::UPPER_BACK,
																						dl_leg::EDiscreteLegPos::UPPER_BACK, dl_leg::EDiscreteLegPos::UPPER_BACK, dl_leg::EDiscreteLegPos::UPPER_BACK };
		res = dl_leg::makeLegState(com_pattern, is_ground, discretized_leg_pos4);
		testLegState(res, com_pattern, is_ground, discretized_leg_pos4);
	}

}	//namespace dl_leg_test