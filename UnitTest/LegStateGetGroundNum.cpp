#pragma once
#include "pch.h"
#include "../DesignLab/LegState.h"

namespace LegStateEditTest
{
	TEST(GroundCheckFunc, GetGroundedLegNum)
	{
		int _leg_state = 0;

		//�S�r�ڒn�C���ʂ�6
		ComType::EComPattern _com_pattern = ComType::EComPattern::front;
		bool _ground[HexapodConst::LEG_NUM] = { true, true, true, true, true, true };
		int _leg_pos[HexapodConst::LEG_NUM] = { 4,4,4,4,4,4 };
		_leg_state = LegStateEdit::makeLegState(_com_pattern, _ground, _leg_pos);

		EXPECT_EQ(LegStateEdit::getGroundedLegNum(_leg_state), 6);

		//�S�r�����C���ʂ�0
		bool _ground2[HexapodConst::LEG_NUM] = { false,false,false,false,false,false };
		_leg_state = LegStateEdit::makeLegState(_com_pattern, _ground2, _leg_pos);

		EXPECT_EQ(LegStateEdit::getGroundedLegNum(_leg_state), 0);

		//5�r�ڒn�C���ʂ�5
		bool _ground3[HexapodConst::LEG_NUM] = { true,true,true,true,true,false };
		_leg_state = LegStateEdit::makeLegState(_com_pattern, _ground3, _leg_pos);

		EXPECT_EQ(LegStateEdit::getGroundedLegNum(_leg_state), 5);

		//1�r�ڒn�C���ʂ�1
		bool _ground4[HexapodConst::LEG_NUM] = { false,false,false,false,false,true };
		_leg_state = LegStateEdit::makeLegState(_com_pattern, _ground4, _leg_pos);

		EXPECT_EQ(LegStateEdit::getGroundedLegNum(_leg_state), 1);

		//4�r�ڒn�C���ʂ�4
		bool _ground5[HexapodConst::LEG_NUM] = { true,true,true,true,false,false };
		_leg_state = LegStateEdit::makeLegState(_com_pattern, _ground5, _leg_pos);

		EXPECT_EQ(LegStateEdit::getGroundedLegNum(_leg_state), 4);

		//2�r�ڒn�C���ʂ�2
		bool _ground6[HexapodConst::LEG_NUM] = { false,false,true,true,false,false };
		_leg_state = LegStateEdit::makeLegState(_com_pattern, _ground6, _leg_pos);

		EXPECT_EQ(LegStateEdit::getGroundedLegNum(_leg_state), 2);

		//3�r�ڒn�C���ʂ�3
		bool _ground7[HexapodConst::LEG_NUM] = { true,true,true,false,false,false };
		_leg_state = LegStateEdit::makeLegState(_com_pattern, _ground7, _leg_pos);

		EXPECT_EQ(LegStateEdit::getGroundedLegNum(_leg_state), 3);
	}

	TEST(GroundCheckFunc, GetLiftedLegNum)
	{
		int _leg_state = 0;

		//�S�r�ڒn�C���ʂ�0
		ComType::EComPattern _com_pattern = ComType::EComPattern::front;
		bool _ground[HexapodConst::LEG_NUM] = { true, true, true, true, true, true };
		int _leg_pos[HexapodConst::LEG_NUM] = { 4,4,4,4,4,4 };
		_leg_state = LegStateEdit::makeLegState(_com_pattern, _ground, _leg_pos);

		EXPECT_EQ(LegStateEdit::getLiftedLegNum(_leg_state), 0);

		//�S�r�����C���ʂ�6
		bool _ground2[HexapodConst::LEG_NUM] = { false,false,false,false,false,false };
		_leg_state = LegStateEdit::makeLegState(_com_pattern, _ground2, _leg_pos);

		EXPECT_EQ(LegStateEdit::getLiftedLegNum(_leg_state), 6);

		//5�r�ڒn�C���ʂ�1
		bool _ground3[HexapodConst::LEG_NUM] = { true,true,true,true,true,false };
		_leg_state = LegStateEdit::makeLegState(_com_pattern, _ground3, _leg_pos);

		EXPECT_EQ(LegStateEdit::getLiftedLegNum(_leg_state), 1);

		//1�r�ڒn�C���ʂ�5
		bool _ground4[HexapodConst::LEG_NUM] = { false,false,false,false,false,true };
		_leg_state = LegStateEdit::makeLegState(_com_pattern, _ground4, _leg_pos);

		EXPECT_EQ(LegStateEdit::getLiftedLegNum(_leg_state), 5);

		//4�r�ڒn�C���ʂ�2
		bool _ground5[HexapodConst::LEG_NUM] = { true,true,true,true,false,false };
		_leg_state = LegStateEdit::makeLegState(_com_pattern, _ground5, _leg_pos);

		EXPECT_EQ(LegStateEdit::getLiftedLegNum(_leg_state), 2);

		//2�r�ڒn�C���ʂ�4
		bool _ground6[HexapodConst::LEG_NUM] = { false,false,true,true,false,false };
		_leg_state = LegStateEdit::makeLegState(_com_pattern, _ground6, _leg_pos);

		EXPECT_EQ(LegStateEdit::getLiftedLegNum(_leg_state), 4);

		//3�r�ڒn�C���ʂ�3
		bool _ground7[HexapodConst::LEG_NUM] = { true,true,true,false,false,false };
		_leg_state = LegStateEdit::makeLegState(_com_pattern, _ground7, _leg_pos);

		EXPECT_EQ(LegStateEdit::getLiftedLegNum(_leg_state), 3);
	}
}
