#include "pch.h"
#include "../DesignLab/LegState.h"
#include "../DesignLab/LegState.cpp"

using namespace leg_state;

TEST(LegStateTest, makeLegStateTest)
{
	//�S�r�ڒn�̃f�[�^���쐬����D
	int _com_pattern = 0;
	bool _ground[HexapodConst::LEG_NUM] = { true, true, true, true, true, true };
	int _leg_pos[HexapodConst::LEG_NUM] = { 4,4,4,4,4,4 };

	EXPECT_EQ(makeLegState(_com_pattern, _ground, _leg_pos), 0b00000000110011001100110011001100);

	//�S�r�V�r�̃f�[�^���쐬����D
	bool _lifted[HexapodConst::LEG_NUM] = { false, false, false, false, false, false };

	EXPECT_EQ(makeLegState(_com_pattern, _lifted, _leg_pos), 0b00000000010001000100010001000100);

	//�d�S�p�^�[���̒l�𐳂������f�ł��邩
	EXPECT_EQ(makeLegState(-2, _ground, _leg_pos),							 0b00000000110011001100110011001100);
	EXPECT_EQ(makeLegState(-1, _ground, _leg_pos),							 0b00000000110011001100110011001100);
	EXPECT_EQ(makeLegState(ComType::COM_PATTERN_NUM, _ground, _leg_pos),	 0b00000000110011001100110011001100);
	EXPECT_EQ(makeLegState(ComType::COM_PATTERN_NUM + 1, _ground, _leg_pos), 0b00000000110011001100110011001100);	//�͈͊O

	EXPECT_EQ(makeLegState(0, _ground, _leg_pos), 0b00000000110011001100110011001100);
	EXPECT_EQ(makeLegState(1, _ground, _leg_pos), 0b00000001110011001100110011001100);
	EXPECT_EQ(makeLegState(2, _ground, _leg_pos), 0b00000010110011001100110011001100);
	EXPECT_EQ(makeLegState(3, _ground, _leg_pos), 0b00000011110011001100110011001100);
	EXPECT_EQ(makeLegState(4, _ground, _leg_pos), 0b00000100110011001100110011001100);
	EXPECT_EQ(makeLegState(5, _ground, _leg_pos), 0b00000101110011001100110011001100);
	EXPECT_EQ(makeLegState(6, _ground, _leg_pos), 0b00000110110011001100110011001100);
	EXPECT_EQ(makeLegState(7, _ground, _leg_pos), 0b00000111110011001100110011001100);
	EXPECT_EQ(makeLegState(8, _ground, _leg_pos), 0b00001000110011001100110011001100);
	EXPECT_EQ(makeLegState(9, _ground, _leg_pos), 0b00001001110011001100110011001100);

	//
}

TEST(LegStateTest, isGroundedTest)
{
	//�S�r�ڒn�̃f�[�^
	const int all_ground_leg_state = 0b11110000100010001000100010001000;

	EXPECT_FALSE(isGrounded(all_ground_leg_state, -1));		// 0 �` 5�@�͈̔͊O�̓��͕͂K��false�ɂȂ�D
	EXPECT_TRUE(isGrounded(all_ground_leg_state, 0));
	EXPECT_TRUE(isGrounded(all_ground_leg_state, 1));
	EXPECT_TRUE(isGrounded(all_ground_leg_state, 2));
	EXPECT_TRUE(isGrounded(all_ground_leg_state, 3));
	EXPECT_TRUE(isGrounded(all_ground_leg_state, 4));
	EXPECT_TRUE(isGrounded(all_ground_leg_state, 5));
	EXPECT_FALSE(isGrounded(all_ground_leg_state, 6));

	//�S�r�V�r�̃f�[�^
	const int all_lifted_leg_state = 0b11110000000000000000000000000000;

	EXPECT_FALSE(isGrounded(all_lifted_leg_state, -1));
	EXPECT_FALSE(isGrounded(all_lifted_leg_state, 0));
	EXPECT_FALSE(isGrounded(all_lifted_leg_state, 1));
	EXPECT_FALSE(isGrounded(all_lifted_leg_state, 2));
	EXPECT_FALSE(isGrounded(all_lifted_leg_state, 3));
	EXPECT_FALSE(isGrounded(all_lifted_leg_state, 4));
	EXPECT_FALSE(isGrounded(all_lifted_leg_state, 5));
	EXPECT_FALSE(isGrounded(all_lifted_leg_state, 6));

	//�r 0 �Ƌr 3 �C�r 5 ���ڒn
	//							    �]��d�S�r�T�r�S�r�R�r�Q�r�P�r�O
	const int sample_leg_state	= 0b11110000100001101001001000001000;
	EXPECT_FALSE(isGrounded(sample_leg_state, -1));
	EXPECT_TRUE(isGrounded(sample_leg_state, 0));
	EXPECT_FALSE(isGrounded(sample_leg_state, 1));
	EXPECT_FALSE(isGrounded(sample_leg_state, 2));
	EXPECT_TRUE(isGrounded(sample_leg_state, 3));
	EXPECT_FALSE(isGrounded(sample_leg_state, 4));
	EXPECT_TRUE(isGrounded(sample_leg_state, 5));
	EXPECT_FALSE(isGrounded(sample_leg_state, 6));
}

TEST(LegStateTest, getGroundedLegNumTest) 
{

}

TEST(LegStateTest, isAbleLegNumTest)
{
	EXPECT_FALSE(isAbleLegNum(-1));	//�͈͊O

	EXPECT_TRUE(isAbleLegNum(0));
	EXPECT_TRUE(isAbleLegNum(1));
	EXPECT_TRUE(isAbleLegNum(2));
	EXPECT_TRUE(isAbleLegNum(3));
	EXPECT_TRUE(isAbleLegNum(4));
	EXPECT_TRUE(isAbleLegNum(5));

	EXPECT_FALSE(isAbleLegNum(6));	//�͈͊O
}
