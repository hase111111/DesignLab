#pragma once

#include <vector>

#include "hexapod_const.h"
#include "com_type.h"


//! @namespace dl_leg
//! @date 2023/08/11
//! @author ���J��
//! @brief �r��Ԃ�ҏW���邽�߂̊֐����܂Ƃ߂����O��ԁDDesignLab Leg Functions�̗��D
//! @details �r��Ԃ��Č����Ă邯�Ǐd�S�^�C�v�������Ă�DC++������int�^��32bit�D
//! @n 1�r�̋r��Ԃ�4bit�ŕ\�� �ŏ��0:�V�r,1:�ڒn�D�@�c��3bit�ŗ��U�������r�ʒu�D
//! @n ���U�������r�ʒu�͊�ʒu��4�Ƃ��āC������O�ɂ���Ȃ�4���傫�������C���ɂ���Ȃ��4��菬���������D @n
//! @n [���U�������r�ʒu] 
//!	@n 7 .. 3    (0�͎g��Ȃ�) 
//! @n 6 4 2  
//! @n 5 .. 1 @n
//! @n [bit�̃f�[�^]  	
//! @n �]��@�d�S�p�^�[���@�r�T�@�r�S�@�r�R�@�r�Q�@�r�P�@�r�O
//! @n 1111 1111�@�@�@�@�@1111�@1111�@1111�@1111�@1111�@1111 @n
//! @n �r�͉E�O�r��0�Ƃ��Ď��v���� 0 �` 5 �D
//! @n �萔�͊�{�I�ɂ͕ҏW���Ȃ��悤�ɁD�r�̗��U�����@��ύX���鎞�ȊO�ҏW����K�v�͂Ȃ��͂��D
//! @n DISCRETE_NUM�͋r�̗��U������\���Ă��܂��D���̒l�͑��̃t�@�C������Q�Ƃ��邱�Ƃ��������C����ȊO�̒l�͑��̃t�@�C������Q�Ƃ��Ȃ��悤�ɁC
//! @n �ǂ����Ă��Q�Ƃ������ꍇ�C���̖��O��ԂɐV�����֐���ǉ�����ׂ��D
namespace dl_leg
{
	constexpr int DISCRETE_NUM = 7;					//!< �r��Ԃ̗��U�����Dleg_state.h�̒ʂ�7�ʂ�D

	constexpr int LEG_POS_MASKBIT = 0b0111;			//!< �r�ʒu��4bit�̉��ʎO���ŊǗ������̂ŁC�������}�X�N����

	constexpr int LEG_GROUNDED_MASKBIT = 0b1000;	//!< �r���ڒn���Ă��邩��\���r�b�g���}�X�N����D(�ڒn���Ă���Ȃ��1�D�V�r�Ȃ��0�D)

	constexpr int LEG_STATE_MASKBIT = 0b1111;		//!< �r��Ԃ�4bit�ŊǗ������̂ŁC�������}�X�N����

	constexpr int SHIFT_TO_COM_NUM = HexapodConst::LEG_NUM * 4;			//!< �d�S�p�^�[����ۑ�����r�b�g�܂ōs�����߂ɁC�ǂꂾ���r�b�g���V�t�g���邩�D

	constexpr int COM_STATE_MASKBIT = (0b1111 << SHIFT_TO_COM_NUM);		//!< �d�S�p�^�[����ۑ�����r�b�g���}�X�N����r�b�g�D


	//! @enum EDiscreteLegPos
	//! @date 2023/09/10
	//! @author ���J��
	//! @breif ���U�����ꂽ�r�ʒu��\��enum
	enum class EDiscreteLegPos
	{
		LOWER_BACK = 1,		//!< ���݂̈ʒu������������ɂ���
		BACK = 2,			//!< ���݂̈ʒu������ɂ���
		UPPER_BACK = 3,		//!< ���݂̈ʒu�����������ɂ���
		CENTER = 4,			//!< ���݂̈ʒu�ɂ���
		LOWER_FRONT = 5,	//!< ���݂̈ʒu���O���������ɂ���
		FRONT = 7,			//!< ���݂̈ʒu���O���ɂ���
		UPPER_FRONT = 6,	//!< ���݂̈ʒu���O��������ɂ���
	};


	//! @brief �r��Ԃ��쐬���ĕԂ��֐��D�r��Ԃ͏d�S�p�^�[���C�r�̐ڒn�E�V�r�C���U�������r�ʒu�̃f�[�^���܂܂��D
	//! @param [in] com_pattern �ǂ̏d�S�p�^�[�����D�ڂ�����ComType.h�ɋL�q�D
	//! @param [in] is_ground �r���ڒn���Ă��邩��\��bool�^�̔z��D�ڒn���Ă���Ȃ��true�D�V�r���Ă���Ȃ��false
	//! @param [in] discretized_leg_pos ���U�������r�ʒu��\���ϐ��D 1 �` 7 �͈̔͂ŗ^����D�͈͊O�Ȃ�Ύ����I�� 4 �ɂȂ�D
	//! @return int �쐬�����r��Ԃ�Ԃ��D
	int makeLegState(ComType::EComPattern com_pattern, const bool is_ground[HexapodConst::LEG_NUM], const int discretized_leg_pos[HexapodConst::LEG_NUM]);


	//! @brief �r�ԍ� leg_index 0 �` 5 �ɉ����āC���̋r���ڒn���Ă��邩�𒲂ׂ�D@n �r�͉E�O�r��0�ԂƂ��āC���v����0,1,2,3,4,5�ƂȂ�D���O����5�ԁD
	//! @param [in] leg_state ���݂̋r���
	//! @param [in] leg_index �ǂ̋r�𒲂ׂ邩�D 0 �` 5 �̐����œ��͂���D
	//! @return bool �r���ڒn���Ă���Ȃ��true��Ԃ��D
	bool isGrounded(int leg_state, int leg_index);


	//! @brief �ڒn���Ă���r�̖{����Ԃ��֐��D
	//! @param [in] leg_state ���݂̋r���
	//! @return int �ڒn���Ă���r�̖{��
	int getGroundedLegNum(int leg_state);


	//! @brief �V�r���Ă���r�̖{����Ԃ��֐��D
	//! @param [in] leg_state ���݂̋r���
	//! @return int �V�r���Ă���r�̖{��
	int getLiftedLegNum(int leg_state);


	//! @brief �ڒn���Ă���r�̋r�ԍ�0�`5���C����res_index�ŎQ�Ɠn������֐�
	//! @param [in] leg_state ���݂̋r���
	//! @param [out] res_index �ڒn���Ă���r�̋r�ԍ����i�[����ϐ��D
	void getGroundedLegNumWithVector(int leg_state, std::vector<int>* res_index);


	//! @brief �V�r���Ă���r�̋r�ԍ�0�`5���C����_res_number�ŎQ�Ɠn������֐�
	//! @param [in] leg_state ���݂̋r���
	//! @param [out] res_index �V�r���Ă���r�̋r�ԍ����i�[����ϐ��D
	void getLiftedLegNumWithVector(int leg_state, std::vector<int>* res_index);


	//! @brief �r��Ԃ��擾����D
	//! @param [in] leg_state ���݂̋r���
	//! @param [in] leg_index �ǂ̋r�̏�Ԃ��擾���邩�D 0 �` 5 �̐����œ��͂���D
	//! @return int �r�̏�Ԃ�Ԃ��D1 �` 7 �̐����D
	int getLegState(int leg_state, int leg_index);


	//! @brief �d�S�p�^�[�����擾����D0b0000 �` 0b1000 �܂�
	//! @param [in] leg_state ���݂̋r���
	//! @return int �d�S�p�^�[����Ԃ��D0b0000 �` 0b1000
	int getComPatternState(int leg_state);


	//! @brief �r�̏��� 0�`15 �̏�ԂɕύX����D�����̒l�����������Ȃ��false���o�͂���D
	//! @param [in] leg_index �r�̔ԍ� 0�`5
	//! @param [in] new_discretized_leg_pos �V�����r��� 1�`15
	//! @param [in,out] leg_state ���݂̋r��ԁC��������ƂɐV�����r��Ԃ��쐬����D
	//! @return bool �ύX�ɐ���������true
	bool changeLegState(int leg_index, int new_discretized_leg_pos, int* leg_state);


	// �r�̏��� 0�`7 �̏�ԂɕύX����D�����̒l�����������Ȃ��false���o�͂���D�V�r��\��bit�͂��̂܂�
	// _leg_state �ύX����r��ԁD_leg_num �ǂ̋r��ύX���邩 0�`5�D_new_state �V�����r��� 1�`7
	bool changeLegStateKeepTopBit(int& _leg_state, int _leg_num, int _new_state);


	//�r�̐ڒn�E�V�r����ύX����D��1������_leg_state���Q�Ɠn�����ĕύX����
	void changeGround(int& _leg_state, int _leg_num, bool _ground);


	//! @brief �d�S�̃f�[�^��ύX����D
	//! @param [in] leg_state ���݂̋r���
	//! @param [in] new_com_pattern �V�����d�S�p�^�[��
	//! @return int �ύX�����d�S�p�^�[����Ԃ��D
	int changeComPattern(int leg_state, ComType::EComPattern new_com_pattern);


	//! @brief �r��\���ԍ��́C�E�O�r�� 0 �Ƃ��āC 0 �` 5 �͈̔́D���͈̔͊O�Ȃ��false��Ԃ��D
	//! @param [in] num �r�̖{��
	//! @return bool 0 �` 5 �͈͓̔��Ȃ��true
	constexpr bool isAbleLegNum(const int num)
	{
		if (0 <= num && num < HexapodConst::LEG_NUM) { return true; }	// 0 �` 5�Ȃ� true

		return false;
	}


	constexpr bool isAbleLegState(const int state)
	{
		if (state == 8) { return false; }		// 8 (0b1000) �Ȃ� false

		if (0 < state && state < 15) { return true; }		// 1 �` 15�Ȃ� true

		return false;
	}

	//! @brief �r�̏㉺�̕ω��񐔂��o�͂���
	//! @param [in] leg_state_first 1�ڂ̋r���
	//! @param [in] leg_state_second 2�ڂ̋r���
	//! @return int �r�̏㉺�̕ω���
	int getLegUpDownCount(int leg_state_first, int leg_state_second);
}


//! @file leg_state.h
//! @date 2023/08/11
//! @author ���J��
//! @brief ���̃v���O�����ł͋r��Ԃ�int�^(32bit)�̏��ŕ\���D���̃f�[�^���������邽�߂̊֐����܂Ƃ߂�����
//! @n �s�� : @lineinfo
