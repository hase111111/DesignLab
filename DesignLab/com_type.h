#pragma once

#include <vector>
#include <string>

#include "hexapod_const.h"
#include "discrete_com_pos.h"


//! @namespace dl_com
//! @date 2023/08/09
//! @author ���J��
//! @brief �d�S�^�C�v�Ɋւ��閼�O���
//! @details leg_state�̏��bit�ɂĕ\����Ă�����́D�ڍׂ͔g������̏C�_��
//! @n BFSinHierarchy�ECreateComCandidate�EPassFinding�Ɨl�X�ȃt�@�C���Ɍׂ鏈�����܂Ƃ߂����č��������. 
//! @n �E�d�S�p�^�[�� �c 10�ʂ肠��Dleg_state �̏��bit�ɂĕ\���������� 
//! @n �E�d�S�^�C�v  �c 36�ʂ肠��Dground_leg ����\�Ȃ��̂ɐ���������U���Ă���D
//!	@n 	�ƁC���J��͒�`�����D�g����y�͂����������S�ă^�C�v(COMType)�Ƃ��ēǂ�ł���̂ŋ�ʂ��邽�߂ɂ�����`����. @n
//! @n �r�̗V�r�̃p�^�[���́C
//! @n �E�S�ڒn  1�ʂ� 
//! @n �E1�{�V�r 6�ʂ� 
//! @n �E2�{�V�r 15�ʂ� 
//! @n �E3�{�V�r 20�ʂ� �� �����\�Ȃ��̂�14�ʂ� 
//!	@n �Ȃ̂őS����36�ʂ肠��D 
//! @note �g�����F���Ȃ̂őS�ʓI�ɏ��������������C�����[�����Ȃ̂ŁC�Ƃ肠�������̂܂܂ɂ��Ă����D
namespace dl_com
{
	constexpr int COM_PATTERN_NUM = 10;		//!< �d�S�p�^�[���̐�

	constexpr int COM_TYPE_NUM = 36;		//!< �d�S�^�C�v�̐�

	constexpr int BAN_LIST_ARRAY_SIZE = 23;

	constexpr char BAN_LIST[HexapodConst::LEG_NUM][BAN_LIST_ARRAY_SIZE] =
	{

	{ 0, 1, 2, 3, 4, 5,       8, 9,10,11,12,   14,   16,      19,20,21,22,      25,   27,28,      31,32,   34    },	//�E�O�r
	{ 0,    2, 3, 4, 5, 6, 7, 8,      11,12,13,   15,   17,      20,21,22,23,      26,   28,29,      32,33,   35 },	//�E���r
	{ 0, 1,    3, 4, 5, 6, 7,    9,10,   12,13,14,   16,   18,      21,22,23,24,      27,   29,30,      33,34    },	//�E��r
	{ 0, 1, 2,    4, 5, 6,    8, 9,   11,   13,14,15,   17,18,19,      22,23,24,25,      28,      31,      34,35 },	//����r
	{ 0, 1, 2, 3,    5, 6, 7, 8,   10,   12,   14,15,16,   18,19,20,      23,   25,26,      29,30,   32,      35 },	//�����r
	{ 0, 1, 2, 3, 4,    6, 7,    9,10,11,   13,   15,   17,18,19,20,21,      24,   26,27,      30,31,   33       }	//���O�r

	};

	const std::vector< std::vector<int> > COMTYPE_BAN_LIST =
	{
		{ 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35 },	//�p�^�[��0 �ǂׂ̗肠���������グ�邱�Ƃ��ł��Ȃ�
		{ 18, 19,             23, 24, 25,             29, 30, 31,             35 },	//�p�^�[��6
		{ 18, 19, 20,             24, 25, 26,             30, 31, 32 },				//�p�^�[��1
		{ 19, 20, 21,             25, 26, 27,             31, 32, 33 },			//�p�^�[��2
		{ 20, 21, 22,             26, 27, 28,             32, 33, 34 },		//�p�^�[��3
		{ 21, 22, 23,             27, 28, 29,             33, 34, 35 },	//�p�^�[��4
		{ 18,             22, 23, 24,             28, 29, 30,             34, 35 },	//�p�^�[��5
		{ 18,     20,     22,     24,     26,     28,     30,     32,     34, },		//�p�^�[��7
		{ 19,     21,     23,     25,     27,     29,     31,     33,     35 }	//�p�^�[��8
	};


	//�ڒn���Ă���r��true�Ƃ���bool�^�̔z��ƁC�d�S�p�^�[������C�\�Ȃ��̂����o�͂���
	bool isAbleCoM(int com_pattern, const bool is_ground[HexapodConst::LEG_NUM]);

	//�ڒn���Ă���r��true�����z�񂩂�C�d�S�^�C�v���o�͂���֐��D�Y�����Ȃ��Ȃ�Ε��̒l��Ԃ�
	char getComTypeFromGroundLeg(const bool is_ground[HexapodConst::LEG_NUM]);

	//�r��Ԃ���C�d�S�^�C�v���o�͂���֐��D�Y�����Ȃ��Ȃ�Ε��̒l��Ԃ�
	char getComTypeFromLegState(int leg_state);

	//�ڒn�r��1�C�V�r��0�Ƃ����r�b�g����C�d�S�^�C�v���o�͂���֐��D�Y�����Ȃ��Ȃ�Ε��̒l��Ԃ�
	char getComTypeFromBit(const int bit);

	//�d�S�^�C�v����C�ڒn�r��1�C�V�r��0�Ƃ����r�b�g���o�͂���֐��D�Y�����Ȃ��Ȃ�ΑS��false��Ԃ��DgetComTypeFromBit�̋t�̏����D
	void getGroundLegFromComType(const int com_type, bool output_ground_leg[HexapodConst::LEG_NUM]);

	// CCC��蓾����com pattern��p���āC�Ƃ肦�Ȃ�com type��vector�ŕԂ�
	void getDonotUseComTypeFromComPattern(const int com_pattern, std::vector<int> output);

	// CCC��蓾����com pattern��p���āC�Ƃ肦�Ȃ�com type�����ׂ�false�ɂ���D_com_type_able_array�͑S36��com type���g�p�\���ǂ�����\��bool�^�̔z��D���̒l��ҏW����D
	void checkAbleComTypeFromComPattern(const int com_pattern, bool com_type_able_array[COM_TYPE_NUM]);

	//�ڒn���邱�Ƃ̂ł��Ȃ��r����C�Ƃ肦�Ȃ�com type�����ׂ�false�ɂ���D_com_type_able_array�͑S36��com type���g�p�\���ǂ�����\��bool�^�̔z��D���̒l��ҏW����D
	void checkAbleComTypeFromNotGroundableLeg(const int not_groundble_leg, bool output_able_comtype[COM_TYPE_NUM]);

	//�V�r���邱�Ƃ̂ł��Ȃ��r����C�Ƃ肦�Ȃ�com type�����ׂ�false�ɂ���D_com_type_able_array�͑S36��com type���g�p�\���ǂ�����\��bool�^�̔z��D���̒l��ҏW����D
	void checkAbleComTypeFromNotFreeLeg(const int not_free_leg_num, bool output_able_comtype[COM_TYPE_NUM]);

} // namespace dl_com



//! @file com_type.h
//! @date 2023/08/09
//! @author ���J��
//! @brief �d�S�^�C�v���������߂̖��O��ԁD
//! @n �s�� : @lineinfo