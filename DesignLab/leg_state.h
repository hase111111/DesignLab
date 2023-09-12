#pragma once

#include <vector>
#include <bitset>

#include "hexapod_const.h"
#include "com_type.h"
#include "discrete_leg_pos.h"


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

	constexpr int LEG_STATE_BIT_NUM = HexapodConst::LEG_NUM * 4 + 4;		//!< �r��Ԃ�ۑ�����r�b�g���D

	constexpr std::bitset<LEG_STATE_BIT_NUM> LEG_POS_MASKBIT(0b0111);		//!< �r�ʒu��4bit�̉��ʎO���ŊǗ������̂ŁC�������}�X�N����

	constexpr std::bitset<LEG_STATE_BIT_NUM> LEG_GROUNDED_MASKBIT(0b1000);	//!< �r���ڒn���Ă��邩��\���r�b�g���}�X�N����D(�ڒn���Ă���Ȃ��1�D�V�r�Ȃ��0�D)

	constexpr std::bitset<LEG_STATE_BIT_NUM> LEG_STATE_MASKBIT(0b1111);		//!< �r��Ԃ�4bit�ŊǗ������̂ŁC�������}�X�N����


	constexpr int SHIFT_TO_COM_NUM = HexapodConst::LEG_NUM * 4;				//!< �d�S�p�^�[����ۑ�����r�b�g�܂ōs�����߂ɁC�ǂꂾ���r�b�g���V�t�g���邩�D

	constexpr std::bitset<LEG_STATE_BIT_NUM> COM_STATE_MASKBIT = (0b1111 << SHIFT_TO_COM_NUM);		//!< �d�S�p�^�[����ۑ�����r�b�g���}�X�N����r�b�g�D



	//! @brief �r��Ԃ��쐬���ĕԂ��֐��D�r��Ԃ͏d�S�p�^�[���C�r�̐ڒn�E�V�r�C���U�������r�ʒu�̃f�[�^���܂܂��D
	//! @param [in] com_pattern �ǂ̏d�S�p�^�[�����D�ڂ�����ComType.h�ɋL�q�D
	//! @param [in] is_ground �r���ڒn���Ă��邩��\��bool�^�̔z��D�ڒn���Ă���Ȃ��true�D�V�r���Ă���Ȃ��false
	//! @param [in] discretized_leg_pos ���U�������r�ʒu��\���ϐ��D
	//! @return std::bitset<LEG_STATE_BIT_NUM> �쐬�����r��Ԃ�Ԃ��D
	std::bitset<LEG_STATE_BIT_NUM> makeLegState(dl_com::EComPattern com_pattern, const bool is_ground[HexapodConst::LEG_NUM], const EDiscreteLegPos discretized_leg_pos[HexapodConst::LEG_NUM]);


	//! @brief �r�ԍ� leg_index 0 �` 5 �ɉ����āC���̋r���ڒn���Ă��邩�𒲂ׂ�D@n �r�͉E�O�r��0�ԂƂ��āC���v����0,1,2,3,4,5�ƂȂ�D���O����5�ԁD
	//! @param [in] leg_state ���݂̋r���
	//! @param [in] leg_index �ǂ̋r�𒲂ׂ邩�D 0 �` 5 �̐����œ��͂���D
	//! @return bool �r���ڒn���Ă���Ȃ��true��Ԃ��D
	bool isGrounded(const std::bitset<LEG_STATE_BIT_NUM>& leg_state, int leg_index);


	//! @brief �ڒn���Ă���r�̖{����Ԃ��֐��D
	//! @param [in] leg_state ���݂̋r���
	//! @return int �ڒn���Ă���r�̖{��
	int getGroundedLegNum(const std::bitset<LEG_STATE_BIT_NUM>& leg_state);


	//! @brief �V�r���Ă���r�̖{����Ԃ��֐��D
	//! @param [in] leg_state ���݂̋r���
	//! @return int �V�r���Ă���r�̖{��
	int getLiftedLegNum(const std::bitset<LEG_STATE_BIT_NUM>& leg_state);


	//! @brief �ڒn���Ă���r�̋r�ԍ�0�`5���C����res_index�ŎQ�Ɠn������֐�
	//! @param [in] leg_state ���݂̋r���
	//! @param [out] res_index �ڒn���Ă���r�̋r�ԍ����i�[����ϐ��D
	void getGroundedLegIndexWithVector(const std::bitset<LEG_STATE_BIT_NUM>& leg_state, std::vector<int>* res_index);


	//! @brief �V�r���Ă���r�̋r�ԍ�0�`5���C����_res_number�ŎQ�Ɠn������֐�
	//! @param [in] leg_state ���݂̋r���
	//! @param [out] res_index �V�r���Ă���r�̋r�ԍ����i�[����ϐ��D
	void getLiftedLegIndexWithVector(const std::bitset<LEG_STATE_BIT_NUM>& leg_state, std::vector<int>* res_index);


	//! @brief �r��Ԃ��擾����D
	//! @param [in] leg_state ���݂̋r���
	//! @param [in] leg_index �ǂ̋r�̏�Ԃ��擾���邩�D 0 �` 5 �̐����œ��͂���D
	//! @return EDiscreteLegPos ���U�����ꂽ�r�̈ʒu��Ԃ��D
	EDiscreteLegPos getLegState(const std::bitset<LEG_STATE_BIT_NUM>& leg_state, int leg_index);


	//! @brief ���݂̋r��Ԃ���d�S�p�^�[�����擾����D
	//! @param [in] leg_state ���݂̋r���
	//! @return dl_com::EComPattern �d�S�p�^�[����Ԃ��D
	dl_com::EComPattern getComPatternState(const std::bitset<LEG_STATE_BIT_NUM>& leg_state);


	//! @brief �r�̏���ύX����D�����̒l�����������Ȃ��false���o�͂���D
	//! @param [in] leg_index �r�̔ԍ� 0�`5
	//! @param [in] new_discretized_leg_pos �V�����r���
	//! @param [in] is_ground �r���ڒn���Ă��邩��\���D�ڒn���Ă���Ȃ��true�D�V�r���Ă���Ȃ��false
	//! @param [in,out] leg_state ���݂̋r��ԁC��������ƂɐV�����r��Ԃ��쐬����D
	//! @return bool �ύX�ɐ���������true
	bool changeLegState(int leg_index, EDiscreteLegPos new_discretized_leg_pos, bool is_ground, std::bitset<LEG_STATE_BIT_NUM>* leg_state);


	//! @brief �r�̏�Ԃ�ύX����D�����̒l�����������Ȃ��false���o�͂���D�V�r��\��bit�͂��̂܂�
	//! @param [in] leg_index �r�̔ԍ� 0�`5
	//! @param [in] new_discretized_leg_pos �V�����r���
	//! @param [in,out] leg_state ���݂̋r��ԁC��������ƂɐV�����r��Ԃ��쐬����D
	//! @return bool �ύX�ɐ���������true
	bool changeLegStateKeepTopBit(int leg_index, EDiscreteLegPos new_discretized_leg_pos, std::bitset<LEG_STATE_BIT_NUM>* leg_state);


	//! @breif �r�̐ڒn�E�V�r����ύX����D
	//! @param [in] leg_index �r�̔ԍ� 0�`5
	//! @param [in] is_ground �r���ڒn���Ă��邩��\���D�ڒn���Ă���Ȃ��true�D�V�r���Ă���Ȃ��false
	//! @param [in,out] leg_state ���݂̋r��ԁC��������ƂɐV�����r��Ԃ��쐬����D
	void changeGround(int leg_index, bool is_ground, std::bitset<LEG_STATE_BIT_NUM>* leg_state);


	//! @brief �d�S�̃f�[�^��ύX����D
	//! @param [in] leg_state ���݂̋r���
	//! @param [in] new_com_pattern �V�����d�S�p�^�[��
	//! @return std::bitset<LEG_STATE_BIT_NUM> �ύX�����d�S�p�^�[����Ԃ��D
	std::bitset<LEG_STATE_BIT_NUM> changeComPattern(const std::bitset<LEG_STATE_BIT_NUM>& leg_state, dl_com::EComPattern new_com_pattern);


	//! @brief �r��\���ԍ��́C�E�O�r�� 0 �Ƃ��āC 0 �` 5 �͈̔́D���͈̔͊O�Ȃ��false��Ԃ��D
	//! @param [in] num �r�̖{��
	//! @return bool 0 �` 5 �͈͓̔��Ȃ��true
	constexpr bool isAbleLegNum(const int num)
	{
		if (0 <= num && num < HexapodConst::LEG_NUM) { return true; }	// 0 �` 5�Ȃ� true

		return false;
	}

	//! @brief �r�̏㉺�̕ω��񐔂��o�͂���
	//! @param [in] leg_state_first 1�ڂ̋r���
	//! @param [in] leg_state_second 2�ڂ̋r���
	//! @return int �r�̏㉺�̕ω���
	int getLegUpDownCount(int leg_state_first, int leg_state_second);

}	// namespace dl_leg



//! @file leg_state.h
//! @date 2023/08/11
//! @author ���J��
//! @brief ���̃v���O�����ł͋r��Ԃ�int�^(32bit)�̏��ŕ\���D���̃f�[�^���������邽�߂̊֐����܂Ƃ߂�����
//! @n �s�� : @lineinfo
