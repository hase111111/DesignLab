#include "pch.h"

#include "../DesignLab/leg_state.h"
#include "../DesignLab/com_type.h"
#include "../DesignLab/com_type.cpp"



namespace LegStateEditTest
{
	TEST(MakeLegStateFunc, MakeLegState_ChangeComPattren)
	{
		//�S�r�ڒn�̃f�[�^���쐬����D
		ComType::EComPattern com_pattern = ComType::EComPattern::FRONT;
		bool _ground[HexapodConst::LEG_NUM] = { true, true, true, true, true, true };
		int _leg_pos[HexapodConst::LEG_NUM] = { 4,4,4,4,4,4 };
		int _expected = 0b110011001100110011001100 | ComType::convertComPatternToBit(com_pattern) << dl_leg::SHIFT_TO_COM_NUM;
		EXPECT_EQ(dl_leg::makeLegState(com_pattern, _ground, _leg_pos), _expected);

		//�d�S�p�^�[����ύX���āC�d�S�̒l���ς�邩�C�r�ʒu�̒l���ω����Ȃ����m�F
		com_pattern = ComType::EComPattern::BACK;
		_expected = 0b110011001100110011001100 | ComType::convertComPatternToBit(com_pattern) << dl_leg::SHIFT_TO_COM_NUM;
		EXPECT_EQ(dl_leg::makeLegState(com_pattern, _ground, _leg_pos), _expected);

		com_pattern = ComType::EComPattern::BACK_LEFT;
		_expected = 0b110011001100110011001100 | ComType::convertComPatternToBit(com_pattern) << dl_leg::SHIFT_TO_COM_NUM;
		EXPECT_EQ(dl_leg::makeLegState(com_pattern, _ground, _leg_pos), _expected);
	}


	TEST(MakeLegStateFunc, MakeLegState_ChangeGround)
	{
		//�l�X�ȋr�ڒn�p�^�[�����쐬����D
		ComType::EComPattern _com_pattern = ComType::EComPattern::FRONT;
		bool _ground[HexapodConst::LEG_NUM] = { false, true, true, true, true, true };
		int _leg_pos[HexapodConst::LEG_NUM] = { 4,4,4,4,4,4 };
		int _expected = 0b110011001100110011000100 | ComType::convertComPatternToBit(_com_pattern) << dl_leg::SHIFT_TO_COM_NUM;
		EXPECT_EQ(dl_leg::makeLegState(_com_pattern, _ground, _leg_pos), _expected);

		_ground[0] = true; _ground[1] = false; _ground[2] = true;
		_ground[3] = true; _ground[4] = true; _ground[5] = true;
		_expected = 0b110011001100110001001100 | ComType::convertComPatternToBit(_com_pattern) << dl_leg::SHIFT_TO_COM_NUM;
		EXPECT_EQ(dl_leg::makeLegState(_com_pattern, _ground, _leg_pos), _expected);

		_ground[0] = true; _ground[1] = true; _ground[2] = false;
		_ground[3] = true; _ground[4] = true; _ground[5] = true;
		_expected = 0b110011001100010011001100 | ComType::convertComPatternToBit(_com_pattern) << dl_leg::SHIFT_TO_COM_NUM;
		EXPECT_EQ(dl_leg::makeLegState(_com_pattern, _ground, _leg_pos), _expected);

		//�S�r�V�r�̃f�[�^���쐬����D
		_ground[0] = false; _ground[1] = false; _ground[2] = false;
		_ground[3] = false; _ground[4] = false; _ground[5] = false;
		_expected = 0b010001000100010001000100 | ComType::convertComPatternToBit(_com_pattern) << dl_leg::SHIFT_TO_COM_NUM;
		EXPECT_EQ(dl_leg::makeLegState(_com_pattern, _ground, _leg_pos), _expected);
	}


	TEST(MakeLegStateFunc, MAkeLegState_ChangeLegPos)
	{
		//�l�X�ȋr�ʒu�p�^�[�����쐬����D
		ComType::EComPattern _com_pattern = ComType::EComPattern::FRONT;
		bool _ground[HexapodConst::LEG_NUM] = { true, true, true, true, true, true };
		int _leg_pos[HexapodConst::LEG_NUM] = { 4,4,4,4,4,4 };
		int _expected = 0b110011001100110011001100 | ComType::convertComPatternToBit(_com_pattern) << dl_leg::SHIFT_TO_COM_NUM;
		EXPECT_EQ(dl_leg::makeLegState(_com_pattern, _ground, _leg_pos), _expected);

		//�r�ʒu��ύX���āC�r�ʒu�̒l���ς�邩�C�d�S�̒l���ω����Ȃ����m�F
		_leg_pos[0] = 3; _leg_pos[1] = 3; _leg_pos[2] = 3;
		_leg_pos[3] = 3; _leg_pos[4] = 3; _leg_pos[5] = 3;
		_expected = 0b101110111011101110111011 | ComType::convertComPatternToBit(_com_pattern) << dl_leg::SHIFT_TO_COM_NUM;
		EXPECT_EQ(dl_leg::makeLegState(_com_pattern, _ground, _leg_pos), _expected);

		_leg_pos[0] = 2; _leg_pos[1] = 2; _leg_pos[2] = 2;
		_leg_pos[3] = 2; _leg_pos[4] = 2; _leg_pos[5] = 2;
		_expected = 0b101010101010101010101010 | ComType::convertComPatternToBit(_com_pattern) << dl_leg::SHIFT_TO_COM_NUM;
		EXPECT_EQ(dl_leg::makeLegState(_com_pattern, _ground, _leg_pos), _expected);

		//�͈͊O�̒l�����Ă��C�͈͓��̒l�Ɏ��܂邩�m�F
		_leg_pos[0] = 10; _leg_pos[1] = 10; _leg_pos[2] = 10;
		_leg_pos[3] = -5; _leg_pos[4] = -5; _leg_pos[5] = -5;
		_expected = 0b110011001100110011001100 | ComType::convertComPatternToBit(_com_pattern) << dl_leg::SHIFT_TO_COM_NUM;
		EXPECT_EQ(dl_leg::makeLegState(_com_pattern, _ground, _leg_pos), _expected);
	}
}