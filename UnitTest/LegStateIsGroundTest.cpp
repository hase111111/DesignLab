#include "pch.h"
#include "../DesignLab/LegState.h"
#include "../DesignLab/LegState.cpp"

using namespace LegStateEdit;

namespace LegStateTest
{
	//�S�Ă̋r���ڒn���Ă��鎞��isGrounded�֐��̋������m�F
	TEST(GroundCheckFunc, IsGround_AllGround)
	{
		//�r�ʒu�S�C�S�Đڒn�̋r��Ԃ��쐬
		int com_pattern = 0;
		bool is_ground[HexapodConst::LEG_NUM] = { true,true,true,true,true,true };
		int leg_pos[HexapodConst::LEG_NUM] = { 4,4,4,4,4,4 };
		int leg_state = makeLegState(com_pattern, is_ground, leg_pos);

		//�S�Đڒn���Ă��邩�m�F
		EXPECT_TRUE(isGrounded(leg_state, 0));
		EXPECT_TRUE(isGrounded(leg_state, 1));
		EXPECT_TRUE(isGrounded(leg_state, 2));
		EXPECT_TRUE(isGrounded(leg_state, 3));
		EXPECT_TRUE(isGrounded(leg_state, 4));
		EXPECT_TRUE(isGrounded(leg_state, 5));


		//�r�ʒu�P�`�V�C�S�Đڒn�̋r��Ԃ��쐬
		int leg_pos2[HexapodConst::LEG_NUM] = { 1,2,3,5,6,7 };
		leg_state = makeLegState(com_pattern, is_ground, leg_pos2);

		//�S�Đڒn���Ă��邩�m�F
		EXPECT_TRUE(isGrounded(leg_state, 0));
		EXPECT_TRUE(isGrounded(leg_state, 1));
		EXPECT_TRUE(isGrounded(leg_state, 2));
		EXPECT_TRUE(isGrounded(leg_state, 3));
		EXPECT_TRUE(isGrounded(leg_state, 4));
		EXPECT_TRUE(isGrounded(leg_state, 5));
	}

	//�S�Ă̋r���ڒn���Ă��Ȃ�����isGrounded�֐��̋������m�F
	TEST(GroundCheckFunc, IsGround_AllNotGround)
	{
		//�r�ʒu�S�C�S�Ĕ�ڒn�̋r��Ԃ��쐬
		int com_pattern = 0;
		bool is_ground[HexapodConst::LEG_NUM] = { false,false,false,false,false,false };
		int leg_pos[HexapodConst::LEG_NUM] = { 4,4,4,4,4,4 };
		int leg_state = makeLegState(com_pattern, is_ground, leg_pos);

		//�S�Ĕ�ڒn���m�F
		EXPECT_FALSE(isGrounded(leg_state, 0));
		EXPECT_FALSE(isGrounded(leg_state, 1));
		EXPECT_FALSE(isGrounded(leg_state, 2));
		EXPECT_FALSE(isGrounded(leg_state, 3));
		EXPECT_FALSE(isGrounded(leg_state, 4));
		EXPECT_FALSE(isGrounded(leg_state, 5));

		//�r�ʒu�P�`�V�C�S�Ĕ�ڒn�̋r��Ԃ��쐬
		int leg_pos2[HexapodConst::LEG_NUM] = { 1,2,3,5,6,7 };
		leg_state = makeLegState(com_pattern, is_ground, leg_pos2);

		//�S�Ĕ�ڒn���m�F
		EXPECT_FALSE(isGrounded(leg_state, 0));
		EXPECT_FALSE(isGrounded(leg_state, 1));
		EXPECT_FALSE(isGrounded(leg_state, 2));
		EXPECT_FALSE(isGrounded(leg_state, 3));
		EXPECT_FALSE(isGrounded(leg_state, 4));
		EXPECT_FALSE(isGrounded(leg_state, 5));
	}

	//�ꕔ�̋r���ڒn���Ă��鎞��isGrounded�֐��̋������m�F
	TEST(GroundCheckFunc, IsGround_SomeGround)
	{
		//�r�ʒu�S�C�S�Ĕ�ڒn�̋r��Ԃ��쐬
		int com_pattern = 0;
		bool is_ground[HexapodConst::LEG_NUM] = { true,false,true,false,false,true };
		int leg_pos[HexapodConst::LEG_NUM] = { 4,4,4,4,4,4 };
		int leg_state = makeLegState(com_pattern, is_ground, leg_pos);

		//�ꕔ�̋r���ڒn���Ă��邩�m�F
		EXPECT_TRUE(isGrounded(leg_state, 0));
		EXPECT_FALSE(isGrounded(leg_state, 1));
		EXPECT_TRUE(isGrounded(leg_state, 2));
		EXPECT_FALSE(isGrounded(leg_state, 3));
		EXPECT_FALSE(isGrounded(leg_state, 4));
		EXPECT_TRUE(isGrounded(leg_state, 5));

		//�ڒn�q��ύX���Ċm�F
		bool is_ground[HexapodConst::LEG_NUM] = { false,true,false,true,true,true };
		int leg_state = makeLegState(com_pattern, is_ground, leg_pos);

		//�ꕔ�̋r���ڒn���Ă��邩�m�F
		EXPECT_FALSE(isGrounded(leg_state, 0));
		EXPECT_TRUE(isGrounded(leg_state, 1));
		EXPECT_FALSE(isGrounded(leg_state, 2));
		EXPECT_TRUE(isGrounded(leg_state, 3));
		EXPECT_TRUE(isGrounded(leg_state, 4));
		EXPECT_TRUE(isGrounded(leg_state, 5));
	}

	//isGrounded�֐��͈̔͊O�̒l����ꂽ���̋������m�F
	TEST(GroundCheckFunc, IsGround_OutOfRange)
	{
		//�r�ʒu�S�C�S�Đڒn�̋r��Ԃ��쐬
		int com_pattern = 0;
		bool is_ground[HexapodConst::LEG_NUM] = { true,true,true,true,true,true };
		int leg_pos[HexapodConst::LEG_NUM] = { 4,4,4,4,4,4 };
		int leg_state = makeLegState(com_pattern, is_ground, leg_pos);

		//�͈͊O�̒l����ꂽ���̋������m�F
		EXPECT_FALSE(isGrounded(leg_state, -2));
		EXPECT_FALSE(isGrounded(leg_state, -1));
		EXPECT_FALSE(isGrounded(leg_state, 6));
		EXPECT_FALSE(isGrounded(leg_state, 7));
	}
}