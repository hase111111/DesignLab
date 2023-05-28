#include "HexapodStateCalculator.h"
#include "MyMath.h"
#include <cmath>

using namespace myvector;

float HexapodStateCalclator::m_leg_rom_r[200] = {};

HexapodStateCalclator::HexapodStateCalclator()
{
	//�W���C���g�̈ʒu������������D
	for (int i = 0; i < HexapodConst::LEG_NUM; i++)
	{
		m_local_femurjoint_pos[i] = m_local_tibiajoint_pos[i] = SVector(0, 0, 0);
	}
}

myvector::SVector HexapodStateCalclator::getGlobalLegPos(const SNode& _node, const int _leg_num) const
{
	return rotVector(getLocalCoxaJointPos(_leg_num) + _node.Leg[_leg_num],_node.rot) + _node.global_center_of_mass;
}

myvector::SVector HexapodStateCalclator::getGlobalCoxaJointPos(const SNode& _node, const int _leg_num) const
{
	return rotVector(getLocalCoxaJointPos(_leg_num), _node.rot) + _node.global_center_of_mass;
}

void HexapodStateCalclator::calclateJointPos(const SNode& _node)
{
	// �t�^���w�I�ɃW���C���g�̏ꏊ���v�Z����D
	//�m�[�h�̋r�ʒu�͐������ꏊ�ɂ���Ƃ����O��̂��ƌv�Z����̂ŁC�߂��Ⴍ����Ȓl���������Ă���Ƃ��܂����삵�Ȃ��D
	//�`�F�b�N����@�\��t����Əd���Ȃ�̂ŁC������������ȃm�[�h�𐶐����Ȃ��悤�ɒ��ӂ���D

	using namespace my_math;

	for (int i = 0; i < HexapodConst::LEG_NUM; i++)
	{
		const float _coxa_joint_angle = atan2(_node.Leg[i].y,_node.Leg[i].x);

		m_local_femurjoint_pos[i] = SVector(HexapodConst::COXA_LENGTH * cos(_coxa_joint_angle), HexapodConst::COXA_LENGTH * sin(_coxa_joint_angle), 0);

		const float _leg_to_coxa_len = sqrt(squared(_node.Leg[i].x) + squared(_node.Leg[i].y));		//�^�ォ�猩���Ƃ��́C�r�悩��r�̕t�����܂ł̒����D
		const float _leg_to_fumur_len = _leg_to_coxa_len - HexapodConst::COXA_LENGTH;				//�^�ォ�猩���Ƃ��́C�r�悩����֐߂܂ł̒����D

		const float _s1 = squared(_leg_to_fumur_len) + squared(HexapodConst::FEMUR_LENGTH) + squared(_node.Leg[i].z) - squared(HexapodConst::TIBIA_LENGTH);
		const float _s2 = 2 * HexapodConst::FEMUR_LENGTH * _leg_to_fumur_len * sqrt(squared(_leg_to_fumur_len) + squared(_node.Leg[i].z));

		const float _fumur_joint_angle = -atan(_leg_to_fumur_len / _node.Leg[i].z) + asin(_s1 / _s2);

		m_local_tibiajoint_pos[i] = m_local_femurjoint_pos[i] + 
										SVector(HexapodConst::FEMUR_LENGTH * cos(_coxa_joint_angle) * cos(_fumur_joint_angle), 
												HexapodConst::FEMUR_LENGTH * sin(_coxa_joint_angle) * cos(_fumur_joint_angle), 
												HexapodConst::FEMUR_LENGTH * sin(_fumur_joint_angle));
	}

}

myvector::SVector HexapodStateCalclator::getGlobalFemurJointPos(const SNode& _node,const int _leg_num) const
{
	return rotVector(getLocalCoxaJointPos(_leg_num) + m_local_femurjoint_pos[_leg_num], _node.rot) + _node.global_center_of_mass;
}

myvector::SVector HexapodStateCalclator::getGlobalTibiaJointPos(const SNode& _node, const int _leg_num) const
{
	return rotVector(getLocalCoxaJointPos(_leg_num) + m_local_tibiajoint_pos[_leg_num], _node.rot) + _node.global_center_of_mass;
}

void HexapodStateCalclator::initLegRomR()
{
	using namespace my_math;

	for (int _z = 0; _z < 200; _z++)
	{
		for (int _x = 53; _x < 248; _x++)
		{
			SVector _tmp_leg((float)_x, 0, -(float)_z);
			_tmp_leg.z += (_tmp_leg.z == 0) ? 0.001f : 0;	

			SVector _local_femurjoint_pos = SVector(HexapodConst::COXA_LENGTH, 0, 0);

			const float _leg_to_coxa_len = sqrt(squared(_tmp_leg.x) + squared(_tmp_leg.y));		//�^�ォ�猩���Ƃ��́C�r�悩��r�̕t�����܂ł̒����D
			float _leg_to_fumur_len = _leg_to_coxa_len - HexapodConst::COXA_LENGTH;				//�^�ォ�猩���Ƃ��́C�r�悩����֐߂܂ł̒����D
			_leg_to_fumur_len += (_leg_to_fumur_len == 0) ? 0.001f : 0;

			const float _s1 = squared(_leg_to_fumur_len) + squared(HexapodConst::FEMUR_LENGTH) + squared(_tmp_leg.z) - squared(HexapodConst::TIBIA_LENGTH);
			const float _s2 = 2 * HexapodConst::FEMUR_LENGTH * _leg_to_fumur_len * sqrt(squared(_leg_to_fumur_len) + squared(_tmp_leg.z));

			const float _fumur_joint_angle = -atan(_leg_to_fumur_len / _tmp_leg.z) + asin(_s1 / _s2);

			SVector _local_tibiajoint_pos = _local_femurjoint_pos +	SVector(HexapodConst::FEMUR_LENGTH * cos(_fumur_joint_angle),
																			0,
																			HexapodConst::FEMUR_LENGTH * sin(_fumur_joint_angle));

			if ((_tmp_leg - _local_tibiajoint_pos).length() < HexapodConst::TIBIA_LENGTH) { m_leg_rom_r[_z] = _x; }
		}
	}


}

myvector::SVector HexapodStateCalclator::getLocalCoxaJointPos(const int _leg_num) const
{
	//�d�S�����_�Ƃ������W�D���{�b�g�̐��ʂ�x�̐��C���{�b�g�̏��z�̐��C�E����W�n��y���W���Ƃ��Ă���D
	//�O���[�o�����W�n��xyz���Ƃ͕ʂ̎��Ȃ̂ŁC��]�͍l������Ă��Ȃ��D

	if (_leg_num == 0)		{ return SVector(HexapodConst::BODY_FRONT_LENGTH,	-HexapodConst::BODY_FRONT_WIDTH,	0.0f); }	// �r0 �E��
	else if (_leg_num == 1) { return SVector(0.0f,								-HexapodConst::BODY_CENTER_WIDTH,	0.0f); }	// �r1 �E��
	else if (_leg_num == 2)	{ return SVector(-HexapodConst::BODY_REAR_LENGTH,	-HexapodConst::BODY_REAR_WIDTH,		0.0f); }	// �r2 �E��
	else if (_leg_num == 3)	{ return SVector(-HexapodConst::BODY_REAR_LENGTH,	HexapodConst::BODY_REAR_WIDTH,		0.0f); }	// �r3 ����
	else if (_leg_num == 4)	{ return SVector(0.0f,								HexapodConst::BODY_CENTER_WIDTH,	0.0f); }	// �r4 ����
	else if (_leg_num == 5)	{ return SVector(HexapodConst::BODY_FRONT_LENGTH,	HexapodConst::BODY_FRONT_WIDTH,		0.0f); }	// �r5 ����

	return myvector::SVector(0, 0, 0);
}