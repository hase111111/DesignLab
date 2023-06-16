#pragma once
#include "HexapodConst.h"
#include <vector>

//
//leg_state�̏��bit�ɂĕ\����Ă�����́D�ڍׂ͔g������̏C�_�ŁD
//BFSinHierarchy�ECreateComCandidate�EPassFinding�Ɨl�X�ȃt�@�C���Ɍׂ鏈�����܂Ƃ߂����č��������.
//
//�E�d�S�p�^�[�� �c 10�ʂ肠��Dleg_state �̏��bit�ɂĕ\����������
//�E�d�S�^�C�v  �c 36�ʂ肠��Dground_leg ����\�Ȃ��̂ɐ���������U���Ă���D
//		�ƁC���J��͒�`�����D�g����y�͂����������S�ă^�C�v(COMType)�Ƃ��ēǂ�ł���̂ŋ�ʂ��邽�߂ɂ�����`����.
//
//�@�r�̗V�r�̃p�^�[���́C
//�@�@�S�ڒn  1�ʂ�
//�@�@1�{�V�r 6�ʂ�
//�@�@2�{�V�r 15�ʂ� �� �����\�Ȃ��̂�??�ʂ�
//�@�@3�{�V�r 20�ʂ� �� �����\�Ȃ��̂�??�ʂ�
//		�Ȃ̂őS����36�ʂ肠��D
//

namespace ComType 
{
	constexpr int COM_PATTERN_NUM = 10;		//�d�S�p�^�[���̐�

	constexpr int COM_TYPE_NUM = 36;		//�d�S�^�C�v�̐�


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
