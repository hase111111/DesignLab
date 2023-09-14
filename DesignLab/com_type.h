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


	//constexpr int COM_PATTERN_NUM = 10;		//!< �d�S�p�^�[���̐�

	//constexpr int COM_TYPE_NUM = 36;		//!< �d�S�^�C�v�̐�

	//constexpr int BAN_LIST_ARRAY_SIZE = 23;

	//constexpr char BAN_LIST[HexapodConst::LEG_NUM][BAN_LIST_ARRAY_SIZE] =
	//{

	//{ 0, 1, 2, 3, 4, 5,       8, 9,10,11,12,   14,   16,      19,20,21,22,      25,   27,28,      31,32,   34    },	//�E�O�r
	//{ 0,    2, 3, 4, 5, 6, 7, 8,      11,12,13,   15,   17,      20,21,22,23,      26,   28,29,      32,33,   35 },	//�E���r
	//{ 0, 1,    3, 4, 5, 6, 7,    9,10,   12,13,14,   16,   18,      21,22,23,24,      27,   29,30,      33,34    },	//�E��r
	//{ 0, 1, 2,    4, 5, 6,    8, 9,   11,   13,14,15,   17,18,19,      22,23,24,25,      28,      31,      34,35 },	//����r
	//{ 0, 1, 2, 3,    5, 6, 7, 8,   10,   12,   14,15,16,   18,19,20,      23,   25,26,      29,30,   32,      35 },	//�����r
	//{ 0, 1, 2, 3, 4,    6, 7,    9,10,11,   13,   15,   17,18,19,20,21,      24,   26,27,      30,31,   33       }	//���O�r

	//};

	//const std::vector< std::vector<int> > COMTYPE_BAN_LIST =
	//{
	//	{ 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35 },	//�p�^�[��0 �ǂׂ̗肠���������グ�邱�Ƃ��ł��Ȃ�
	//	{ 18, 19,             23, 24, 25,             29, 30, 31,             35 },	//�p�^�[��6
	//	{ 18, 19, 20,             24, 25, 26,             30, 31, 32 },				//�p�^�[��1
	//	{ 19, 20, 21,             25, 26, 27,             31, 32, 33 },				//�p�^�[��2
	//	{ 20, 21, 22,             26, 27, 28,             32, 33, 34 },				//�p�^�[��3
	//	{ 21, 22, 23,             27, 28, 29,             33, 34, 35 },				//�p�^�[��4
	//	{ 18,             22, 23, 24,             28, 29, 30,             34, 35 },	//�p�^�[��5
	//	{ 18,     20,     22,     24,     26,     28,     30,     32,     34, },	//�p�^�[��7
	//	{ 19,     21,     23,     25,     27,     29,     31,     33,     35 }		//�p�^�[��8
	//};


	////�d�S�^�C�v����C�ڒn�r��1�C�V�r��0�Ƃ����r�b�g���o�͂���֐��D�Y�����Ȃ��Ȃ�ΑS��false��Ԃ��DgetComTypeFromBit�̋t�̏����D
	//void getGroundLegFromComType(const int com_type, bool output_ground_leg[HexapodConst::LEG_NUM]);

	//// CCC��蓾����com pattern��p���āC�Ƃ肦�Ȃ�com type��vector�ŕԂ�
	//void getDonotUseComTypeFromComPattern(const int com_pattern, std::vector<int>& output);

	//// CCC��蓾����com pattern��p���āC�Ƃ肦�Ȃ�com type�����ׂ�false�ɂ���D_com_type_able_array�͑S36��com type���g�p�\���ǂ�����\��bool�^�̔z��D���̒l��ҏW����D
	//void checkAbleComTypeFromComPattern(const int com_pattern, bool com_type_able_array[COM_TYPE_NUM]);

	////�ڒn���邱�Ƃ̂ł��Ȃ��r����C�Ƃ肦�Ȃ�com type�����ׂ�false�ɂ���D_com_type_able_array�͑S36��com type���g�p�\���ǂ�����\��bool�^�̔z��D���̒l��ҏW����D
	//void checkAbleComTypeFromNotGroundableLeg(const int not_groundble_leg, bool output_able_comtype[COM_TYPE_NUM]);

	////�V�r���邱�Ƃ̂ł��Ȃ��r����C�Ƃ肦�Ȃ�com type�����ׂ�false�ɂ���D_com_type_able_array�͑S36��com type���g�p�\���ǂ�����\��bool�^�̔z��D���̒l��ҏW����D
	//void checkAbleComTypeFromNotFreeLeg(const int not_free_leg_num, bool output_able_comtype[COM_TYPE_NUM]);

} // namespace dl_com



//! @file com_type.h
//! @date 2023/08/09
//! @author ���J��
//! @brief �d�S�^�C�v���������߂̖��O��ԁD
//! @n �s�� : @lineinfo