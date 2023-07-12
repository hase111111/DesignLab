#include "pch.h"
#include "../DesignLab/LegState.h"
#include "../DesignLab/LegState.cpp"

using namespace leg_state;

namespace LegStateTest
{
	//全ての脚が接地している時のisGrounded関数の挙動を確認
	TEST(GroundCheckFunc, IsGround_AllGround)
	{
		//脚位置４，全て接地の脚状態を作成
		int com_pattern = 0;
		bool is_ground[HexapodConst::LEG_NUM] = { true,true,true,true,true,true };
		int leg_pos[HexapodConst::LEG_NUM] = { 4,4,4,4,4,4 };
		int leg_state = makeLegState(com_pattern, is_ground, leg_pos);

		//全て接地しているか確認
		EXPECT_TRUE(isGrounded(leg_state, 0));
		EXPECT_TRUE(isGrounded(leg_state, 1));
		EXPECT_TRUE(isGrounded(leg_state, 2));
		EXPECT_TRUE(isGrounded(leg_state, 3));
		EXPECT_TRUE(isGrounded(leg_state, 4));
		EXPECT_TRUE(isGrounded(leg_state, 5));


		//脚位置１〜７，全て接地の脚状態を作成
		int leg_pos2[HexapodConst::LEG_NUM] = { 1,2,3,5,6,7 };
		leg_state = makeLegState(com_pattern, is_ground, leg_pos2);

		//全て接地しているか確認
		EXPECT_TRUE(isGrounded(leg_state, 0));
		EXPECT_TRUE(isGrounded(leg_state, 1));
		EXPECT_TRUE(isGrounded(leg_state, 2));
		EXPECT_TRUE(isGrounded(leg_state, 3));
		EXPECT_TRUE(isGrounded(leg_state, 4));
		EXPECT_TRUE(isGrounded(leg_state, 5));
	}

	//全ての脚が接地していない時のisGrounded関数の挙動を確認
	TEST(GroundCheckFunc, IsGround_AllNotGround)
	{
		//脚位置４，全て非接地の脚状態を作成
		int com_pattern = 0;
		bool is_ground[HexapodConst::LEG_NUM] = { false,false,false,false,false,false };
		int leg_pos[HexapodConst::LEG_NUM] = { 4,4,4,4,4,4 };
		int leg_state = makeLegState(com_pattern, is_ground, leg_pos);

		//全て非接地か確認
		EXPECT_FALSE(isGrounded(leg_state, 0));
		EXPECT_FALSE(isGrounded(leg_state, 1));
		EXPECT_FALSE(isGrounded(leg_state, 2));
		EXPECT_FALSE(isGrounded(leg_state, 3));
		EXPECT_FALSE(isGrounded(leg_state, 4));
		EXPECT_FALSE(isGrounded(leg_state, 5));

		//脚位置１〜７，全て非接地の脚状態を作成
		int leg_pos2[HexapodConst::LEG_NUM] = { 1,2,3,5,6,7 };
		leg_state = makeLegState(com_pattern, is_ground, leg_pos2);

		//全て非接地か確認
		EXPECT_FALSE(isGrounded(leg_state, 0));
		EXPECT_FALSE(isGrounded(leg_state, 1));
		EXPECT_FALSE(isGrounded(leg_state, 2));
		EXPECT_FALSE(isGrounded(leg_state, 3));
		EXPECT_FALSE(isGrounded(leg_state, 4));
		EXPECT_FALSE(isGrounded(leg_state, 5));
	}

	//一部の脚が接地している時のisGrounded関数の挙動を確認
	TEST(GroundCheckFunc, IsGround_SomeGround)
	{
		//脚位置４，全て非接地の脚状態を作成
		int com_pattern = 0;
		bool is_ground[HexapodConst::LEG_NUM] = { true,false,true,false,false,true };
		int leg_pos[HexapodConst::LEG_NUM] = { 4,4,4,4,4,4 };
		int leg_state = makeLegState(com_pattern, is_ground, leg_pos);

		//一部の脚が接地しているか確認
		EXPECT_TRUE(isGrounded(leg_state, 0));
		EXPECT_FALSE(isGrounded(leg_state, 1));
		EXPECT_TRUE(isGrounded(leg_state, 2));
		EXPECT_FALSE(isGrounded(leg_state, 3));
		EXPECT_FALSE(isGrounded(leg_state, 4));
		EXPECT_TRUE(isGrounded(leg_state, 5));

		//接地客を変更して確認
		bool is_ground[HexapodConst::LEG_NUM] = { false,true,false,true,true,true };
		int leg_state = makeLegState(com_pattern, is_ground, leg_pos);

		//一部の脚が接地しているか確認
		EXPECT_FALSE(isGrounded(leg_state, 0));
		EXPECT_TRUE(isGrounded(leg_state, 1));
		EXPECT_FALSE(isGrounded(leg_state, 2));
		EXPECT_TRUE(isGrounded(leg_state, 3));
		EXPECT_TRUE(isGrounded(leg_state, 4));
		EXPECT_TRUE(isGrounded(leg_state, 5));
	}

	//isGrounded関数の範囲外の値を入れた時の挙動を確認
	TEST(GroundCheckFunc, IsGround_OutOfRange)
	{
		//脚位置４，全て接地の脚状態を作成
		int com_pattern = 0;
		bool is_ground[HexapodConst::LEG_NUM] = { true,true,true,true,true,true };
		int leg_pos[HexapodConst::LEG_NUM] = { 4,4,4,4,4,4 };
		int leg_state = makeLegState(com_pattern, is_ground, leg_pos);

		//範囲外の値を入れた時の挙動を確認
		EXPECT_FALSE(isGrounded(leg_state, -2));
		EXPECT_FALSE(isGrounded(leg_state, -1));
		EXPECT_FALSE(isGrounded(leg_state, 6));
		EXPECT_FALSE(isGrounded(leg_state, 7));
	}
}