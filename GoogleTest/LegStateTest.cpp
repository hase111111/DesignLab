#include "pch.h"
#include "LegState.h"
#include "LegState.cpp"

using namespace LegState;

//isGrounded�̓�����m�F����e�X�g
TEST(LegState, is_grounded) 
{
	//4bit�̏�ʂ�1�Ȃ�ڒn�C0�Ȃ�V�r
	int _all_ground = 0b11111111100010001000100010001000;

	EXPECT_TRUE(isGrounded(_all_ground, 0));
	EXPECT_TRUE(isGrounded(_all_ground, 1));
	EXPECT_TRUE(isGrounded(_all_ground, 2));
	EXPECT_TRUE(isGrounded(_all_ground, 3));
	EXPECT_TRUE(isGrounded(_all_ground, 4));
	EXPECT_TRUE(isGrounded(_all_ground, 5));

	EXPECT_FALSE(isGrounded(_all_ground, -1));
	EXPECT_FALSE(isGrounded(_all_ground, 6));

	// 0��4�͗V�r
	int _0_4_lift = 0b11111111100001111000100010000111;

	EXPECT_FALSE(isGrounded(_0_4_lift, 0));
	EXPECT_TRUE(isGrounded(_0_4_lift, 1));
	EXPECT_TRUE(isGrounded(_0_4_lift, 2));
	EXPECT_TRUE(isGrounded(_0_4_lift, 3));
	EXPECT_FALSE(isGrounded(_0_4_lift, 4));
	EXPECT_TRUE(isGrounded(_0_4_lift, 5));
}

//changeLegState�̓�����m�F����e�X�g
TEST(LegState, change_leg_state)
{
	int _all_state_zero = 0b11111111100010001000100010001000;
	int _state = 0b0100;

	EXPECT_TRUE(changeLegState(_all_state_zero, 0, _state));
	EXPECT_EQ(_all_state_zero, 0b11111111100010001000100010000100);

	EXPECT_TRUE(changeLegState(_all_state_zero, 3, _state));
	EXPECT_EQ(_all_state_zero, 0b11111111100010000100100010000100);

	//�r�̋��E���e�X�g
	EXPECT_FALSE(changeLegState(_all_state_zero, -1, _state));
	EXPECT_FALSE(changeLegState(_all_state_zero, 6, _state));

	//��Ԃ̋��E���e�X�g
	EXPECT_FALSE(changeLegState(_all_state_zero, 0, 0b0000));
	EXPECT_FALSE(changeLegState(_all_state_zero, 0, 0b1000));
	EXPECT_FALSE(changeLegState(_all_state_zero, 0, 0b10000));

	//�ω��͂Ȃ��͂�
	EXPECT_EQ(_all_state_zero, 0b11111111100010000100100010000100);
}

//changeLegStateKeepTopBit�̓�����m�F����e�X�g
TEST(LegState, change_leg_state_keep_top)
{
	int _all_state_zero = 0b11111111100010001000100010001000;
	int _state = 0b0100;

	//0�Ԃ̋r��_state�̒l�ɕύX
	EXPECT_TRUE(changeLegStateKeepTopBit(_all_state_zero, 0, _state));
	EXPECT_EQ(_all_state_zero, 0b11111111100010001000100010001100);

	//3�Ԃ̋r��_state�̒l�ɕύX
	EXPECT_TRUE(changeLegStateKeepTopBit(_all_state_zero, 3, _state));
	EXPECT_EQ(_all_state_zero, 0b11111111100010001100100010001100);

	//�r�̋��E���e�X�g
	EXPECT_FALSE(changeLegStateKeepTopBit(_all_state_zero, -1, _state));
	EXPECT_FALSE(changeLegStateKeepTopBit(_all_state_zero, 6, _state));

	//��Ԃ̋��E���e�X�g
	EXPECT_FALSE(changeLegStateKeepTopBit(_all_state_zero, 0, 0b0000));
	EXPECT_FALSE(changeLegStateKeepTopBit(_all_state_zero, 0, 0b1000));
	EXPECT_FALSE(changeLegStateKeepTopBit(_all_state_zero, 0, 0b10000));

	//�ω��͂Ȃ��͂�
	EXPECT_EQ(_all_state_zero, 0b11111111100010001100100010001100);
}
