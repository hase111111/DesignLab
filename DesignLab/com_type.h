#pragma once

#include <vector>
#include <unordered_map>
#include <bitset>

#include <boost/bimap.hpp>
#include <boost/dynamic_bitset.hpp>

#include "hexapod_const.h"
#include "discrete_com_pos.h"
#include "leg_state.h"


namespace std
{
	// boost bimap�̂��߂ɔ�r���Z�q���`���Ă���D�኱�׈��ȉ����@
	template <size_t N>
	bool operator<(const std::bitset<N>& lhs, const std::bitset<N>& rhs)
	{
		return lhs.to_ulong() < rhs.to_ulong();
	}
}


//! @namespace dl_com
//! @date 2023/08/09
//! @author ���J��
//! @brief �d�S�^�C�v�Ɋւ��閼�O��ԁDDesignLab Center of Mass Positon Functions �̗��D
//! @details leg_state�̏��bit�ɂĕ\����Ă�����́D�ڍׂ͔g������̏C�_��
//! @n BFSinHierarchy�ECreateComCandidate�EPassFinding�Ɨl�X�ȃt�@�C���Ɍׂ鏈�����܂Ƃ߂����č��������. 
//! @n �d�S�ʒu�͂������藣�U�����āC8�ʂ�C
//! @n �r�̐ڒn�p�^�[����36�ʂ肠��D���U�������d�S�ʒu�����邱�Ƃ��ł��Ȃ����̂�\�ߎ�菜���̂������֐��̖����D@n
//! @n �r�̐ڒn�p�^�[���́C
//! @n �E�S�ڒn  1�ʂ� 
//! @n �E1�{�V�r 6�ʂ� 
//! @n �E2�{�V�r 15�ʂ� 
//! @n �E3�{�V�r 20�ʂ� �� �����\�Ȃ��̂�14�ʂ� 
//!	@n �Ȃ̂őS����36�ʂ肠��D 
namespace dl_com
{

	//�O�q�̒ʂ�r�̐ڒn�p�^�[����36�ʂ肠��D���ꂼ��̗V�r�ɑΉ����鐔�l������U���ĊǗ������₷�����邽�߂ɁCbimap��p���Ă���D

	using LegGroundedMap = boost::bimaps::bimap<dl_leg::LegGroundedBit, int>;		//!< �r�̐ڒn�p�^�[����\���^�Dleft��bit�̃f�[�^�Cright��int�^�̔ԍ��D

	using LegGroundedMapValue = LegGroundedMap::value_type;							//!< �r�̐ڒn�p�^�[����\���}�b�v�̒l�̌^�D

	//! @brief leg_index�� leg_index + 1 �Ԃ̋r���Ƃ��ɗV�r�ɂȂ鎞��true��Ԃ��֐��D�������p�Ɏg�p���Ă���
	//! @param leg_index �r�̔ԍ��D
	//! @param leg_ground_pattern_index �r�̐ڒn�p�^�[���̔ԍ��D
	//! @return leg_index�� leg_index + 1 �Ԃ̋r���Ƃ��ɗV�r�ɂȂ鎞��true�D
	bool isLegPairFree(int leg_index, int leg_ground_pattern_index);

	//! @brief �r�̐ڒn�p�^�[����\���}�b�v���쐬����֐��D���������Ɉ�x�����Ăяo���D
	LegGroundedMap makeLegGroundedMap();

	//! @brief �d�S�ʒu����g�p�s�\�Ȑڒn�p�^�[�����쐬����֐��D���������Ɉ�x�����Ăяo���D
	std::unordered_map<EDiscreteComPos, std::vector<int>> makeLegGroundedPatternBanList();

	//! @brief ����̋r���ڒn�ł��Ȃ��ꍇ�Ɏ�蓾�Ȃ��ڒn�p�^�[�����쐬����֐��́D���������Ɉ�x�����Ăяo���D 
	std::vector<std::vector<int>> makeLegGroundedPatternBanListFromLeg();



	//! @brief �r�̐ڒn�p�^�[���̑�����Ԃ��D
	//! @return int �r�̐ڒn�p�^�[���̑����D
	int getLegGroundPatternNum();


	//! @brief �r�̐ڒn�p�^�[���̔ԍ�����C���̋r�̐ڒn�p�^�[����Ԃ��D
	//! @param [in] leg_ground_pattern_index �r�̐ڒn�p�^�[���̔ԍ��D
	//! @return dl_leg::LegGroundedBit �r�̐ڒn�p�^�[���D
	dl_leg::LegGroundedBit getLegGroundedBitFromLegGroundPatternIndex(int leg_ground_pattern_index);


	//! @brief ���U�����ꂽ�d�S�ʒu����C���̏d�S�ʒu�ł͎�蓾�Ȃ��r�ڒn�p�^�[����false�ɂ���D
	//! @param [in] discrete_com_pos ���U�����ꂽ�d�S�ʒu�D
	//! @param [in,out] output �r�ڒn�p�^�[�������s�\�Ȃ��true�C�s�\�Ȃ�false�ɂ���bool�^�̔z��D�d�S�ʒu�ł͎�蓾�Ȃ��r�ڒn�p�^�[����false�ɕύX����D
	void banLegGroundPatternFromCom(EDiscreteComPos discrete_com_pos, boost::dynamic_bitset<>* output);


	//! @breif �ڒn�ł��Ȃ��r�ԍ�����C���̋r���ڒn�ł��Ȃ��ꍇ�Ɏ�蓾�Ȃ��ڒn�p�^�[����false�ɂ���D
	//! @param [in] not_groundble_leg_index �ڒn�ł��Ȃ��r�ԍ��D
	//! @param [in,out] output �r�ڒn�p�^�[�������s�\�Ȃ��true�C�s�\�Ȃ�false�ɂ���bool�^�̔z��D�ڒn�ł��Ȃ��r���ڒn�ł��Ȃ��ꍇ�Ɏ�蓾�Ȃ��ڒn�p�^�[����false�ɕύX����D
	void banLegGroundPatternFromNotGroundableLeg(int not_groundble_leg_index, boost::dynamic_bitset<>* output);


	//! @brief �V�r�ł��Ȃ��r�ԍ�����C���̋r���V�r�ł��Ȃ��ꍇ�Ɏ�蓾�Ȃ��ڒn�p�^�[����false�ɂ���D
	//! @param [in] not_lift_leg_index �V�r�ł��Ȃ��r�ԍ��D
	//! @param [in,out] output �r�ڒn�p�^�[�������s�\�Ȃ��true�C�s�\�Ȃ�false�ɂ���bool�^�̔z��D�V�r�ł��Ȃ��r���V�r�ł��Ȃ��ꍇ�Ɏ�蓾�Ȃ��ڒn�p�^�[����false�ɕύX����D
	void banLegGroundPatternFromNotFreeLeg(int not_lift_leg_index, boost::dynamic_bitset<>* output);

} // namespace dl_com



//! @file com_type.h
//! @date 2023/08/09
//! @author ���J��
//! @brief �d�S�^�C�v���������߂̖��O��ԁD
//! @n �s�� : @lineinfo