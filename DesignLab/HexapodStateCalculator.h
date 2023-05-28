#pragma once
#include "vectorFunc.h"
#include "Node.h"

//���{�b�g�̍��W���ԂȂǂ̒l���v�Z����N���X�D�� Hexapod�N���X�̐i����
class HexapodStateCalclator
{
public:

	//�r���W�͋r�̕t���������_�Ƃ������W�n�Ȃ̂ŁC������O���[�o�����W�ɕϊ�����D�܂��C���{�b�g�̉�]���l������D
	myvector::SVector getGlobalLegPos(const SNode& _node, const int _leg_num) const;	

	// coxa joint (�r�̕t����)�̍��W��Ԃ��D��]���l�������O���[�o�����W.
	myvector::SVector getGlobalCoxaJointPos(const SNode& _node, const int _leg_num) const;	


private:
	myvector::SVector getLocalCoxaJointPos(const int _leg_num) const;	// coxa joint (�r�̕t����)�̍��W��Ԃ��D�d�S�����_�Ƃ��郍�[�J�����W�D
};
