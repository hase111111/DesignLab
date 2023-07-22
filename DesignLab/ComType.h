#pragma once
#include "HexapodConst.h"
#include <vector>

namespace ComType
{
	constexpr int COM_PATTERN_NUM = 10;		//!< �d�S�p�^�[���̐�

	constexpr int COM_TYPE_NUM = 36;		//!< �d�S�^�C�v�̐�

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