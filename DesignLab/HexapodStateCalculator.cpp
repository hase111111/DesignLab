#include "HexapodStateCalculator.h"

using namespace myvector;

myvector::SVector HexapodStateCalclator::getGlobalLegPos(const SNode& _node, const int _leg_num) const
{
	return rotVector(getLocalCoxaJointPos(_leg_num) + _node.Leg[_leg_num],_node.rot) + _node.global_center_of_mass;
}

myvector::SVector HexapodStateCalclator::getGlobalCoxaJointPos(const SNode& _node, const int _leg_num) const
{
	return rotVector(getLocalCoxaJointPos(_leg_num), _node.rot) + _node.global_center_of_mass;
}

myvector::SVector HexapodStateCalclator::getLocalCoxaJointPos(const int _leg_num) const
{
	//�d�S�����_�Ƃ������W�D���{�b�g�̐��ʂ�x�̐��C���{�b�g�̏��z�̐��C�E����W�n��y���W���Ƃ��Ă���D
	//�O���[�o�����W�n��xyz���Ƃ͕ʂ̎��Ȃ̂ŁC��]�͍l������Ă��Ȃ��D

	if (_leg_num == 0)		{ return myvector::VGet(HexapodConst::BODY_FRONT_LENGTH,	-HexapodConst::BODY_FRONT_WIDTH,	0.0f); }	// �r0 �E��
	else if (_leg_num == 1) { return myvector::VGet(0.0f,								-HexapodConst::BODY_CENTER_WIDTH,	0.0f); }	// �r1 �E��
	else if (_leg_num == 2)	{ return myvector::VGet(-HexapodConst::BODY_REAR_LENGTH,	-HexapodConst::BODY_REAR_WIDTH,		0.0f); }	// �r2 �E��
	else if (_leg_num == 3)	{ return myvector::VGet(-HexapodConst::BODY_REAR_LENGTH,	HexapodConst::BODY_REAR_WIDTH,		0.0f); }	// �r3 ����
	else if (_leg_num == 4)	{ return myvector::VGet(0.0f,								HexapodConst::BODY_CENTER_WIDTH,	0.0f); }	// �r4 ����
	else if (_leg_num == 5)	{ return myvector::VGet(HexapodConst::BODY_FRONT_LENGTH,	HexapodConst::BODY_FRONT_WIDTH,		0.0f); }	// �r5 ����

	return myvector::SVector();
}
