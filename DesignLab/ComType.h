#pragma once
#include "Define.h"

//
//leg_state�̏��bit�ɂĕ\����Ă�����́D�ڍׂ͔g������̏C�_�ŁD
//BFSinHierarchy�ECreateComCandidate�EPassFinding�Ɨl�X�ȃt�@�C���Ɍׂ鏈�����܂Ƃ߂����č��������.
//
// �d�S�p�^�[�� �c 10�ʂ肠��Dleg_state �̏��bit�ɂĕ\����������
//  �d�S�^�C�v  �c 35�ʂ肠��Dground_leg ����\�Ȃ��̂ɐ���������U���Ă���D
//		�ƁC���J��͒�`�����D�g����y�͂����������S�ă^�C�v(COMType)�Ƃ��ēǂ�ł���̂ŋ�ʂ��邽�߂ɂ�����`����.
//

namespace ComType 
{
	//�ڒn���Ă���r��true�Ƃ���bool�^�̔z��ƁC�d�S�p�^�[������C�\�Ȃ��̂����o�͂���
	bool isAbleCoM(const int _com_pattern, const bool _ground_leg[Define::LEG_NUM]);

	//�ڒn���Ă���r��true�����z�񂩂�C�d�S�^�C�v���o�͂���֐��D�Y�����Ȃ��Ȃ�Ε��̒l��Ԃ�
	char getComTypeFromGroundLeg(const bool _ground_leg[Define::LEG_NUM]);
}
