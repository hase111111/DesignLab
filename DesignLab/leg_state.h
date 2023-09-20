#pragma once

#include <array>
#include <bitset>
#include <vector>

#include "hexapod_const.h"
#include "discrete_com_pos.h"
#include "discrete_leg_pos.h"


//! @namespace dl_leg
//! @date 2023/09/13
//! @author ���J��
//! @brief �r��Ԃ�ҏW���邽�߂̊֐����܂Ƃ߂����O��ԁDDesignLab Leg Functions�̗��D
//! @details �r��Ԃ��Č����Ă邯�Ǘ��U�����ꂽ�d�S�ʒu�������Ă�D
//! @n 1�r�̋r��Ԃ�4bit�ŕ\�� �ŏ��0:�V�r,1:�ڒn�D�@�c��3bit�ŗ��U�������r�ʒu�D
//! @n ���U�������r�ʒu�͊�ʒu��4�Ƃ��āC������O�ɂ���Ȃ�4���傫�������C���ɂ���Ȃ��4��菬���������D @n
//! @n [���U�������r�ʒu] 
//!	@n 7 .. 3    (0�͎g��Ȃ�) 
//! @n 6 4 2  
//! @n 5 .. 1 @n
//! @n [bit�̃f�[�^]  	
//! @n �d�S�p�^�[���@�r�T�@�r�S�@�r�R�@�r�Q�@�r�P�@�r�O
//! @n 1111�@�@�@�@�@1111�@1111�@1111�@1111�@1111�@1111 @n
//! @n �r�͉E�O�r��0�Ƃ��Ď��v���� 0 �` 5 �D
//! @n �萔�͊�{�I�ɂ͕ҏW���Ȃ��悤�ɁD�r�̗��U�����@��ύX���鎞�ȊO�ҏW����K�v�͂Ȃ��͂��D
namespace dl_leg
{
	//�g�p����^�̒�`

	constexpr int LGE_POS_BIT_NUM = 4;									//!< �r�ʒu��\���r�b�g���D���U�����ꂽ�r�ʒu��3bit�C�V�r�E�ڒn��1bit�D���킹��4bit

	constexpr int COM_POS_BIT_NUM = 4;									//!< �d�S�p�^�[����\���r�b�g���D

	constexpr int LEG_STATE_BIT_NUM = HexapodConst::LEG_NUM * LGE_POS_BIT_NUM + COM_POS_BIT_NUM;	//!< �r��Ԃ�ۑ�����r�b�g���D28bit

	using LegStateBit = std::bitset<LEG_STATE_BIT_NUM>;					//!< �r��Ԃ�ۑ�����^�D28bit�̃r�b�g�^

	using LegGroundedBit = std::bitset<HexapodConst::LEG_NUM>;			//!< �r�̗V�r�E�ڒn��\���^�D6bit�̃r�b�g�^�D�ڒn�� 1 �V�r�� 0�D



	//�r�b�g���}�X�N���邽�߂̒萔

	constexpr LegStateBit LEG_POS_MASKBIT(0b0111);		//!< �r�ʒu��4bit�̉��ʎO���ŊǗ������̂ŁC�������}�X�N����

	constexpr LegStateBit LEG_GROUNDED_MASKBIT(0b1000);	//!< �r���ڒn���Ă��邩��\���r�b�g���}�X�N����D(�ڒn���Ă���Ȃ��1�D�V�r�Ȃ��0�D)

	constexpr LegStateBit LEG_STATE_MASKBIT(0b1111);	//!< �r��Ԃ�4bit�ŊǗ������̂ŁC�������}�X�N����


	constexpr int SHIFT_TO_COM_NUM = HexapodConst::LEG_NUM * 4;				//!< �d�S�p�^�[����ۑ�����r�b�g�܂ōs�����߂ɁC�ǂꂾ���r�b�g���V�t�g���邩�D

	constexpr LegStateBit COM_STATE_MASKBIT = (0b1111 << SHIFT_TO_COM_NUM);	//!< �d�S�p�^�[����ۑ�����r�b�g���}�X�N����r�b�g�D



	//! @brief �r��Ԃ��쐬���ĕԂ��֐��D�r��Ԃ͏d�S�p�^�[���C�r�̐ڒn�E�V�r�C���U�������r�ʒu�̃f�[�^���܂܂��D
	//! @param [in] discrete_com_pos �ǂ̏d�S�p�^�[�����D�ڂ����� com_type.h �ɋL�q�D
	//! @param [in] is_ground �r���ڒn���Ă��邩��\��bool�^�̔z��D�ڒn���Ă���Ȃ��true�D�V�r���Ă���Ȃ��false
	//! @param [in] discretized_leg_pos ���U�������r�ʒu��\���ϐ��D
	//! @return LegStateBit �쐬�����r��Ԃ�Ԃ��D
	LegStateBit MakeLegStateBit(EDiscreteComPos discrete_com_pos, const std::array<bool, HexapodConst::LEG_NUM> &is_ground, 
		const std::array<EDiscreteLegPos, HexapodConst::LEG_NUM>& discretized_leg_pos);


	//! @brief �r�ԍ� leg_index 0 �` 5 �ɉ����āC���̋r���ڒn���Ă��邩�𒲂ׂ�D
	//! @n �r�͉E�O�r��0�ԂƂ��āC���v����0,1,2,3,4,5�ƂȂ�D���O����5�ԁD
	//! @param [in] leg_state_bit ���݂̋r���
	//! @param [in] leg_index �ǂ̋r�𒲂ׂ邩�D 0 �` 5 �̐����œ��͂���D
	//! @return bool �r���ڒn���Ă���Ȃ��true��Ԃ��D
	bool IsGrounded(const LegStateBit& leg_state_bit, int leg_index);

	//! @brief �r���ڒn���Ă���Ȃ�1�C�V�r��0�Ƃ���bit�ŗV�r�E�ڒn�r�̏�Ԃ�Ԃ��D
	//! @n �Ⴆ�� 0 �ԋr�݂̂��V�r���Ă���Ȃ� 0b111 110 ��Ԃ��D
	//! @param [in] leg_state ���݂̋r���
	//! @return LegGroundedBit �r���ڒn���Ă���Ȃ�1�C�V�r��0�Ƃ���bit�ŗV�r�E�ڒn�r�̏�Ԃ�Ԃ��D
	LegGroundedBit GetLegGroundedBit(const LegStateBit& leg_state);

	//! @brief �ڒn���Ă���r�̖{����Ԃ��֐��D
	//! @param [in] leg_state ���݂̋r���
	//! @return int �ڒn���Ă���r�̖{��
	int GetGroundedLegNum(const LegStateBit& leg_state);

	//! @brief �V�r���Ă���r�̖{����Ԃ��֐��D
	//! @param [in] leg_state ���݂̋r���
	//! @return int �V�r���Ă���r�̖{��
	int GetLiftedLegNum(const LegStateBit& leg_state);

	//! @brief �ڒn���Ă���r�̋r�ԍ�0�`5���C����res_index�ŎQ�Ɠn������֐�
	//! @param [in] leg_state ���݂̋r���
	//! @param [out] res_index �ڒn���Ă���r�̋r�ԍ����i�[����ϐ��D
	void GetGroundedLegIndexByVector(const LegStateBit& leg_state, std::vector<int>* res_index);

	//! @brief �V�r���Ă���r�̋r�ԍ�0�`5���C����_res_number�ŎQ�Ɠn������֐�
	//! @param [in] leg_state ���݂̋r���
	//! @param [out] res_index �V�r���Ă���r�̋r�ԍ����i�[����ϐ��D
	void GetLiftedLegIndexByVector(const LegStateBit& leg_state, std::vector<int>* res_index);


	//! @brief �r��Ԃ��擾����D
	//! @param [in] leg_state ���݂̋r���
	//! @param [in] leg_index �ǂ̋r�̏�Ԃ��擾���邩�D 0 �` 5 �̐����œ��͂���D
	//! @return EDiscreteLegPos ���U�����ꂽ�r�̈ʒu��Ԃ��D
	EDiscreteLegPos getLegState(const LegStateBit& leg_state, int leg_index);

	//! @brief ���݂̋r��Ԃ���d�S�p�^�[�����擾����D
	//! @param [in] leg_state ���݂̋r���
	//! @return EDiscreteComPos �d�S�p�^�[����Ԃ��D
	EDiscreteComPos getComPatternState(const LegStateBit& leg_state);


	//! @brief �r�̏���ύX����D�����̒l�����������Ȃ��false���o�͂���D
	//! @param [in] leg_index �r�̔ԍ� 0�`5
	//! @param [in] new_discretized_leg_pos �V�����r���
	//! @param [in] is_ground �r���ڒn���Ă��邩��\���D�ڒn���Ă���Ȃ��true�D�V�r���Ă���Ȃ��false
	//! @param [in,out] leg_state ���݂̋r��ԁC��������ƂɐV�����r��Ԃ��쐬����D
	//! @return bool �ύX�ɐ���������true
	bool changeLegState(int leg_index, EDiscreteLegPos new_discretized_leg_pos, bool is_ground, LegStateBit* leg_state);

	//! @brief �r�̏�Ԃ�ύX����D�����̒l�����������Ȃ��false���o�͂���D�V�r��\��bit�͂��̂܂�
	//! @param [in] leg_index �r�̔ԍ� 0�`5
	//! @param [in] new_discretized_leg_pos �V�����r���
	//! @param [in,out] leg_state ���݂̋r��ԁC��������ƂɐV�����r��Ԃ��쐬����D
	//! @return bool �ύX�ɐ���������true
	bool changeLegStateKeepTopBit(int leg_index, EDiscreteLegPos new_discretized_leg_pos, LegStateBit* leg_state);

	//! @breif �r�̐ڒn�E�V�r����ύX����D
	//! @param [in] leg_index �r�̔ԍ� 0�`5
	//! @param [in] is_ground �r���ڒn���Ă��邩��\���D�ڒn���Ă���Ȃ��true�D�V�r���Ă���Ȃ��false
	//! @param [in,out] leg_state ���݂̋r��ԁC��������ƂɐV�����r��Ԃ��쐬����D
	void changeGround(int leg_index, bool is_ground, LegStateBit* leg_state);

	//! @brief �S�Ă̋r�̐ڒn�E�V�r����ύX����D
	//! @param [in] is_ground_list �r���ڒn���Ă��邩��\���D�ڒn���Ă���Ȃ��true�D�V�r���Ă���Ȃ��false
	//! @param [in,out] leg_state ���݂̋r��ԁC��������ƂɐV�����r��Ԃ��쐬����D
	void changeAllLegGround(const LegGroundedBit& is_ground_list, LegStateBit* leg_state);

	//! @brief �d�S�̃f�[�^��ύX����D
	//! @param [in] leg_state ���݂̋r���
	//! @param [in] new_com_pattern �V�����d�S�p�^�[��
	//! @return LegStateBit �ύX�����d�S�p�^�[����Ԃ��D
	void changeComPattern(EDiscreteComPos new_com_pattern, LegStateBit* leg_state);

}	// namespace dl_leg



//! @file leg_state.h
//! @date 2023/08/11
//! @author ���J��
//! @brief ���̃v���O�����ł͋r��Ԃ�int�^(32bit)�̏��ŕ\���D���̃f�[�^���������邽�߂̊֐����܂Ƃ߂�����
//! @n �s�� : @lineinfo
