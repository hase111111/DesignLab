#pragma once
#include "HexapodConst.h"
#include <vector>

namespace ComType
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

	//! @enum �d�S�p�^�[���̗񋓌^
	//! @brief �d�S���ǂ��ɂ��邩��\���D
	enum class EComPattern : char
	{
		front,			//!< �d�S���O���ɂ���
		left_front,		//!< �d�S�����O���ɂ���
		left_back,		//!< �d�S��������ɂ���
		back,			//!< �d�S������ɂ���
		right_back,		//!< �d�S���E����ɂ���
		right_front,	//!< �d�S���E�O���ɂ���
		center_front,	//!< �d�S�������O���ɂ���D�O�p�`
		center_back,	//!< �d�S����������ɂ���D�t�O�p�a
		Error			//!< �G���[
	};

	//! @brief �d�S�p�^�[����bit�ɕϊ�����֐�
	//! @param[in] _com_pattern �d�S�p�^�[��
	//! @return �d�S�p�^�[����bit�ɕϊ������l
	//! @details �d�S�p�^�[����leg_state�̏��bit�ɂĕ\������Ă��� <br>
	//! 4bit�Ɏ��܂�悤�ɒl��ϊ�����D
	//! @attention �d�S�p�^�[���̐����ς�����ꍇ�́C���̊֐����ύX����K�v������D
	int convertComPatternToBit(const EComPattern _com_pattern);

	//! @brief �d�S�p�^�[����bit����ϊ�����֐�
	//! @param[in] _bit �d�S�p�^�[����bit�ɕϊ������l
	//! @return �d�S�p�^�[��
	EComPattern convertBitToComPattern(const int _bit);

	//�ڒn���Ă���r��true�Ƃ���bool�^�̔z��ƁC�d�S�p�^�[������C�\�Ȃ��̂����o�͂���
	bool isAbleCoM(const int _com_pattern, const bool _ground_leg[HexapodConst::LEG_NUM]);

	//�ڒn���Ă���r��true�����z�񂩂�C�d�S�^�C�v���o�͂���֐��D�Y�����Ȃ��Ȃ�Ε��̒l��Ԃ�
	char getComTypeFromGroundLeg(const bool _ground_leg[HexapodConst::LEG_NUM]);

	//�r��Ԃ���C�d�S�^�C�v���o�͂���֐��D�Y�����Ȃ��Ȃ�Ε��̒l��Ԃ�
	char getComTypeFromLegState(const int _leg_state);

	//�ڒn�r��1�C�V�r��0�Ƃ����r�b�g����C�d�S�^�C�v���o�͂���֐��D�Y�����Ȃ��Ȃ�Ε��̒l��Ԃ�
	char getComTypeFromBit(const int _bit);

	//�d�S�^�C�v����C�ڒn�r��1�C�V�r��0�Ƃ����r�b�g���o�͂���֐��D�Y�����Ȃ��Ȃ�ΑS��false��Ԃ��DgetComTypeFromBit�̋t�̏����D
	void getGroundLegFromComType(const int _com_type, bool _output_ground_leg[HexapodConst::LEG_NUM]);

	// CCC��蓾����com pattern��p���āC�Ƃ肦�Ȃ�com type��vector�ŕԂ�
	void getDonotUseComTypeFromComPattern(const int _com_pattern, std::vector<int> _output);

	// CCC��蓾����com pattern��p���āC�Ƃ肦�Ȃ�com type�����ׂ�false�ɂ���D_com_type_able_array�͑S36��com type���g�p�\���ǂ�����\��bool�^�̔z��D���̒l��ҏW����D
	void checkAbleComTypeFromComPattern(const int _com_pattern, bool _com_type_able_array[COM_TYPE_NUM]);

	//�ڒn���邱�Ƃ̂ł��Ȃ��r����C�Ƃ肦�Ȃ�com type�����ׂ�false�ɂ���D_com_type_able_array�͑S36��com type���g�p�\���ǂ�����\��bool�^�̔z��D���̒l��ҏW����D
	void checkAbleComTypeFromNotGroundableLeg(const int _not_groundble_leg, bool _com_type_able_array[COM_TYPE_NUM]);

	//�V�r���邱�Ƃ̂ł��Ȃ��r����C�Ƃ肦�Ȃ�com type�����ׂ�false�ɂ���D_com_type_able_array�͑S36��com type���g�p�\���ǂ�����\��bool�^�̔z��D���̒l��ҏW����D
	void checkAbleComTypeFromNotFreeLeg(const int _not_free_leg, bool _com_type_able_array[COM_TYPE_NUM]);
}

//! @namespace ComType
//! @brief �d�S�^�C�v�Ɋւ��閼�O���
//! @details leg_state�̏��bit�ɂĕ\����Ă�����́D�ڍׂ͔g������̏C�_�ŁC<br>
//! BFSinHierarchy�ECreateComCandidate�EPassFinding�Ɨl�X�ȃt�@�C���Ɍׂ鏈�����܂Ƃ߂����č��������.<br>
//! <br>
//!�E�d�S�p�^�[�� �c 10�ʂ肠��Dleg_state �̏��bit�ɂĕ\���������� <br>
//!�E�d�S�^�C�v  �c 36�ʂ肠��Dground_leg ����\�Ȃ��̂ɐ���������U���Ă���D<br>
//!		�ƁC���J��͒�`�����D�g����y�͂����������S�ă^�C�v(COMType)�Ƃ��ēǂ�ł���̂ŋ�ʂ��邽�߂ɂ�����`����.<br>
//! <br>
//!�@�r�̗V�r�̃p�^�[���́C<br>
//!�@�@�S�ڒn  1�ʂ� <br>
//!�@�@1�{�V�r 6�ʂ� <br>
//!�@�@2�{�V�r 15�ʂ� <br>
//!�@�@3�{�V�r 20�ʂ� �� �����\�Ȃ��̂�14�ʂ� <br>
//!		�Ȃ̂őS����36�ʂ肠��D <br>
//! @note �g�����F���Ȃ̂őS�ʓI�ɏ��������������C�����[�����Ȃ̂ŁC�Ƃ肠�������̂܂܂ɂ��Ă����D
//! @date 2023/07/11