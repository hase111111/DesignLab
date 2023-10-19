#include "phantomx_state_calculator.h"

#include <cmath>

#include "cassert_define.h"
#include "designlab_line_segment2.h"
#include "designlab_math_util.h"
#include "phantomx_const.h"

namespace dlm = designlab::math_util;


PhantomXStateCalclator::PhantomXStateCalclator() :
	free_leg_pos_({ {
		{170 * cos(PhantomXConst::kCoxaDefaultAngle[0]),170 * sin(PhantomXConst::kCoxaDefaultAngle[0]),-25},
		{170 * cos(PhantomXConst::kCoxaDefaultAngle[1]),170 * sin(PhantomXConst::kCoxaDefaultAngle[1]),-25},
		{170 * cos(PhantomXConst::kCoxaDefaultAngle[2]),170 * sin(PhantomXConst::kCoxaDefaultAngle[2]),-25},
		{170 * cos(PhantomXConst::kCoxaDefaultAngle[3]),170 * sin(PhantomXConst::kCoxaDefaultAngle[3]),-25},
		{170 * cos(PhantomXConst::kCoxaDefaultAngle[4]),170 * sin(PhantomXConst::kCoxaDefaultAngle[4]),-25},
		{170 * cos(PhantomXConst::kCoxaDefaultAngle[5]),170 * sin(PhantomXConst::kCoxaDefaultAngle[5]),-25}
	} })
{
	//�r�̕t�����̈ʒu������������
	local_leg_base_pos_[0] = designlab::Vector3{ PhantomXConst::kCoxaBaseOffsetX, -PhantomXConst::kCoxaBaseOffsetY, PhantomXConst::kCoxaBaseOffsetZ };	// �r0 �E��
	local_leg_base_pos_[1] = designlab::Vector3{ 0.0f, -PhantomXConst::kCenterCoxaBaseOffsetY, PhantomXConst::kCoxaBaseOffsetZ };						// �r1 �E��
	local_leg_base_pos_[2] = designlab::Vector3{ -PhantomXConst::kCoxaBaseOffsetX, -PhantomXConst::kCoxaBaseOffsetY, PhantomXConst::kCoxaBaseOffsetZ };// �r2 �E��
	local_leg_base_pos_[3] = designlab::Vector3{ -PhantomXConst::kCoxaBaseOffsetX, PhantomXConst::kCoxaBaseOffsetY, PhantomXConst::kCoxaBaseOffsetZ };	// �r3 ����
	local_leg_base_pos_[4] = designlab::Vector3{ 0.0f, PhantomXConst::kCenterCoxaBaseOffsetY, PhantomXConst::kCoxaBaseOffsetZ };						// �r4 ����
	local_leg_base_pos_[5] = designlab::Vector3{ PhantomXConst::kCoxaBaseOffsetX, PhantomXConst::kCoxaBaseOffsetY, PhantomXConst::kCoxaBaseOffsetZ };	// �r5 ����


	// is_able_leg_pos_ ������������D������4�d���[�v
	for (int i = 0; i < HexapodConst::kLegNum; i++)
	{
		for (int x = 0; x < kLegPosDivNum; x++)
		{
			for (int y = 0; y < kLegPosDivNum; y++)
			{
				for (int z = 0; z < kLegPosDivNum; z++)
				{
					is_able_leg_pos_[i][x][y][z] = InitIsAbleLegPos(i, x, y, z);
				}
			}
		}
	}
}


void PhantomXStateCalclator::CalculateAllJointState(const RobotStateNode& node, std::array<HexapodJointState, HexapodConst::kLegNum>* joint_state) const
{
	assert(joint_state != nullptr);		//joint_state��nullptr�ł͂Ȃ��D

	//�t�^���w�̎���Reference/Hexapod�̉摜���Q�Ƃ��Ă�������

	//�v�Z���s���D
	for (int i = 0; i < HexapodConst::kLegNum; i++)
	{
		CalculateLocalJointState(i, node.leg_pos[i], &joint_state->at(i));

		(*joint_state)[i].global_joint_position.clear();
		(*joint_state)[i].global_joint_position.resize(4);

		(*joint_state)[i].global_joint_position[0] = node.global_center_of_mass + designlab::rotVector(GetLocalLegBasePosition(i), node.rot);

		if (joint_state->at(i).local_joint_position[1].has_value()) 
		{
			(*joint_state)[i].global_joint_position[1] = node.global_center_of_mass + designlab::rotVector(GetLocalLegBasePosition(i) + joint_state->at(i).local_joint_position[1].value(), node.rot);
		}

		if (joint_state->at(i).local_joint_position[2].has_value()) 
		{
			(*joint_state)[i].global_joint_position[2] = node.global_center_of_mass + designlab::rotVector(GetLocalLegBasePosition(i) + joint_state->at(i).local_joint_position[2].value(), node.rot);
		}

		(*joint_state)[i].global_joint_position[3] = node.global_center_of_mass + designlab::rotVector(GetLocalLegBasePosition(i) + node.leg_pos[i], node.rot);
	}
}


designlab::Vector3 PhantomXStateCalclator::ConvertGlobalToLegPosition(const int leg_index, const designlab::Vector3& leg_pos, const designlab::Vector3& global_center_of_mass, const designlab::EulerXYZ& robot_rot, const bool consider_rot) const
{
	if (consider_rot)
	{
		return designlab::rotVector(leg_pos - global_center_of_mass, robot_rot * -1) - GetLocalLegBasePosition(leg_index);
	}
	else
	{
		return leg_pos - global_center_of_mass - GetLocalLegBasePosition(leg_index);
	}
}

designlab::Vector3 PhantomXStateCalclator::GetFreeLegPosition(const int leg_index) const
{
	assert(0 <= leg_index && leg_index < HexapodConst::kLegNum);	//leg_index�� 0�`5 �ł���D

	return free_leg_pos_[leg_index];
}


designlab::Vector3 PhantomXStateCalclator::GetLocalLegBasePosition(const int leg_index) const
{
	assert(0 <= leg_index && leg_index < HexapodConst::kLegNum);	//leg_index�� 0�`5 �ł���D

	return local_leg_base_pos_[leg_index];
}

designlab::Vector3 PhantomXStateCalclator::GetLocalLegPosition(const int leg_index, const designlab::Vector3& leg_pos) const
{
	assert(0 <= leg_index && leg_index < HexapodConst::kLegNum);	//leg_index�� 0�`5 �ł���D

	return leg_pos + GetLocalLegBasePosition(leg_index);
}


designlab::Vector3 PhantomXStateCalclator::GetGlobalLegBasePosition(const int leg_index, const designlab::Vector3& global_center_of_mass, const designlab::EulerXYZ& robot_rot, const bool consider_rot) const
{
	assert(0 <= leg_index && leg_index < HexapodConst::kLegNum);	//leg_index�� 0�`5 �ł���D

	if (consider_rot) { return designlab::rotVector(GetLocalLegBasePosition(leg_index), robot_rot) + global_center_of_mass; }
	else { return GetLocalLegBasePosition(leg_index) + global_center_of_mass; }
}


designlab::Vector3 PhantomXStateCalclator::GetGlobalLegPosition(const int leg_index, const designlab::Vector3& leg_pos, const designlab::Vector3& global_center_of_mass, const designlab::EulerXYZ& robot_rot, const bool consider_rot) const
{
	assert(0 <= leg_index && leg_index < HexapodConst::kLegNum);	//leg_index�� 0�`5 �ł���D

	if (consider_rot) { return designlab::rotVector(GetLocalLegBasePosition(leg_index) + leg_pos, robot_rot) + global_center_of_mass; }
	else { return global_center_of_mass + GetLocalLegBasePosition(leg_index) + leg_pos; }
}


bool PhantomXStateCalclator::IsLegInRange(const int leg_index, const designlab::Vector3& leg_pos) const
{
	assert(0 <= leg_index && leg_index < HexapodConst::kLegNum);	//leg_index�� 0�`5 �ł���D

	//�͈͊O�Ȃ�false
	if (GetLegPosIndex(leg_pos.x) < 0 || kLegPosDivNum <= GetLegPosIndex(leg_pos.x)) { return false; }
	if (GetLegPosIndex(leg_pos.y) < 0 || kLegPosDivNum <= GetLegPosIndex(leg_pos.y)) { return false; }
	if (GetLegPosIndex(leg_pos.z) < 0 || kLegPosDivNum <= GetLegPosIndex(leg_pos.z)) { return false; }

	if (is_able_leg_pos_[leg_index][GetLegPosIndex(leg_pos.x)][GetLegPosIndex(leg_pos.y)][GetLegPosIndex(leg_pos.z)])
	{
		const designlab::Vector2 leg_pos_xy = leg_pos.ProjectedXY();
		const designlab::Vector2 min_leg_pos_xy{HexapodConst::MOVABLE_LEG_RANGE_COS_MIN[leg_index], HexapodConst::MOVABLE_LEG_RANGE_SIN_MAX[leg_index]};
		const designlab::Vector2 max_leg_pos_xy{HexapodConst::MOVABLE_LEG_RANGE_COS_MAX[leg_index], HexapodConst::MOVABLE_LEG_RANGE_SIN_MIN[leg_index]};

		//�r�̊p�x���͈͓��ɂ��邩���ׂ�D�O�όv�Z�ŊԂɂ��邩���ׂ�
		if (min_leg_pos_xy.Cross(leg_pos_xy) > 0.0f) { return false; }
		if (max_leg_pos_xy.Cross(leg_pos_xy) < 0.0f) { return false; }

		return true;
	}

	return false;
}


bool PhantomXStateCalclator::IsLegInterfering(const std::array<designlab::Vector3, HexapodConst::kLegNum>& leg_pos) const
{
	//�d�S�����_�Ƃ����C���W�n�ɂ����āC�r�̊��𒲂ׂ�D

	//�r�̊��𒲂ׂ�D
	designlab::Vector2 leg_pos_xy[HexapodConst::kLegNum];
	designlab::Vector2 joint_pos_xy[HexapodConst::kLegNum];

	for (int i = 0; i < HexapodConst::kLegNum; i++)
	{
		joint_pos_xy[i] = GetLocalLegBasePosition(i).ProjectedXY();
		leg_pos_xy[i] = leg_pos[i].ProjectedXY() + joint_pos_xy[i];
	}

	//�ׂ̋r�Ƃ̊��𒲂ׂ�D
	for (int i = 0; i < HexapodConst::kLegNum; i++)
	{
		designlab::LineSegment2 line1(joint_pos_xy[i], leg_pos_xy[i]);
		designlab::LineSegment2 line2(joint_pos_xy[(i + 1) % HexapodConst::kLegNum], leg_pos_xy[(i + 1) % HexapodConst::kLegNum]);

		if (line1.HasIntersection(line2)) { return true; }
	}

	return false;
}


bool PhantomXStateCalclator::InitIsAbleLegPos(const int leg_index, const int x, const int y, const int z) const
{
	assert(0 <= leg_index && leg_index < HexapodConst::kLegNum);	//leg_index�� 0�`5 �ł���D

	float x_pos = (kLegPosMax - kLegPosMin) / kLegPosDivNum * x + kLegPosMin;
	float y_pos = (kLegPosMax - kLegPosMin) / kLegPosDivNum * y + kLegPosMin;
	float z_pos = (kLegPosMax - kLegPosMin) / kLegPosDivNum * z + kLegPosMin;

	designlab::Vector3 leg_pos{x_pos, y_pos, z_pos};		//�r��̈ʒu


	//�� x=0 y=0 �̎���false�ɂ���
	if (abs(x_pos) <= (kLegPosMax - kLegPosMin) / (kLegPosDivNum - 1.0f) && abs(y_pos) <= (kLegPosMax - kLegPosMin) / (kLegPosDivNum - 1.0f)) { return false; }

	if (leg_pos.ProjectedXY().GetLength() < kMinLegR) { return false; }

	// �֐߂̊p�x���v�Z����
	HexapodJointState joint_state;

	CalculateLocalJointState(leg_index, leg_pos, &joint_state);

	//�l�̐��ƗL�������m�F����
	if (joint_state.local_joint_position.size() != 4) { return false; }
	if (joint_state.joint_angle.size() != 3) { return false; }

	for (int i = 0; i < 4; i++) 
	{
		if (!joint_state.local_joint_position[i].has_value()) { return false; }
	}

	for (int i = 0; i < 3; i++)
	{
		if (!joint_state.joint_angle[i].has_value()) { return false; }
	}

	//�ȉ��C���X�璷�ȃR�[�h�ɂȂ��Ă���D�폜�\�D
	
	// coxa�֐߂͈͓̔��ɑ��݂��Ă��邩���m�F����
	if (! PhantomXConst::IsVaildCoxaAngle(leg_index, joint_state.joint_angle[0].value())) { return false; }

	// femur�֐߂͈͓̔��ɑ��݂��Ă��邩���m�F����
	if (! PhantomXConst::IsVaildFemurAngle(joint_state.joint_angle[1].value())) { return false; }

	// tibia�֐߂͈͓̔��ɑ��݂��Ă��邩���m�F����
	if (! PhantomXConst::IsVaildTibiaAngle(joint_state.joint_angle[2].value())) { return false; }

	// �����N�̒������m�F����
	if (!dlm::IsEqual((joint_state.local_joint_position[0].value() - joint_state.local_joint_position[1].value()).GetLength(), 
		PhantomXConst::kCoxaLength)) 
	{
		return false; 
	}

	if (!dlm::IsEqual((joint_state.local_joint_position[1].value() - joint_state.local_joint_position[2].value()).GetLength(), 
		PhantomXConst::kFemurLength)) 
	{
		return false; 
	}

	if (!dlm::IsEqual((joint_state.local_joint_position[2].value() - joint_state.local_joint_position[3].value()).GetLength(), 
		PhantomXConst::kTibiaLength)) 
	{
		return false; 
	}

	return true;
}


void PhantomXStateCalclator::CalculateLocalJointState(const int leg_index, const designlab::Vector3& leg_pos, HexapodJointState* joint_state) const
{
	assert(0 <= leg_index && leg_index < HexapodConst::kLegNum);	//leg_index�� 0�`5 �ł���D


	//�e�p�����[�^������������
	const int kJointNum = 4;
	const int kJointAngleNum = kJointNum - 1;

	(*joint_state).local_joint_position.clear();
	(*joint_state).joint_angle.clear();

	(*joint_state).local_joint_position.resize(kJointNum);
	(*joint_state).joint_angle.resize(kJointAngleNum);


	// coxa joint�̌v�Z
	(*joint_state).local_joint_position[0] = designlab::Vector3{ 0, 0, 0 };	//�r���W�n�ł�coxa joint�͌��_�ɂ���D

	// �r��̒ǉ�
	(*joint_state).local_joint_position[3] = leg_pos;

	// coxa angle�̌v�Z
	{
		float coxa_joint_angle = std::atan2f(leg_pos.y, leg_pos.x);

		if (leg_pos.y == 0 && leg_pos.x == 0) { coxa_joint_angle = PhantomXConst::kCoxaDefaultAngle[leg_index]; }

		if (! PhantomXConst::IsVaildCoxaAngle(leg_index, coxa_joint_angle))
		{
			//�͈͊O�Ȃ�΁C180�x��]���������ɔ͈͓��ɂ��邩�𒲂ׂ�D
			if (PhantomXConst::IsVaildCoxaAngle(leg_index, coxa_joint_angle + dlm::kFloatPi))
			{
				coxa_joint_angle += dlm::kFloatPi;
			}
			else if (PhantomXConst::IsVaildCoxaAngle(leg_index, coxa_joint_angle - dlm::kFloatPi))
			{
				coxa_joint_angle -= dlm::kFloatPi;
			}
		}

		(*joint_state).joint_angle[0] = coxa_joint_angle;
	}

	// femur joint�̌v�Z
	{
		const designlab::Vector3 femur_joint_pos = designlab::Vector3{
			PhantomXConst::kCoxaLength * std::cos((*joint_state).joint_angle[0].value()),
			PhantomXConst::kCoxaLength * std::sin((*joint_state).joint_angle[0].value()),
			0
		};

		(*joint_state).local_joint_position[1] = femur_joint_pos;

		// ���������r�悪�r�̕t��������͂��Ȃ��ꍇ�͌v�Z���s��Ȃ�
		if ((leg_pos - femur_joint_pos).GetLength() > PhantomXConst::kFemurLength + PhantomXConst::kTibiaLength) { return; }
	}


	// femur angle �̌v�Z
	const designlab::Vector3 femur_to_leg = leg_pos - (*joint_state).local_joint_position[1].value();		//�r�悩����֐߂܂ł̒����D
	const float femur_to_leg_x = femur_to_leg.ProjectedXY().GetLength() * 																//�r��֌�����������x�̐������ɂ�����W�n�ɒu�������� 			
	((leg_pos.ProjectedXY().GetSquaredLength() > (*joint_state).local_joint_position[1].value().ProjectedXY().GetSquaredLength()) ? 1.f : -1.f);	//�r�悪���֐߂����߂��ꍇ�͐��̕����ɂ���D
	const float femur_to_leg_z = femur_to_leg.z;

	{
		const float arccos_upper = femur_to_leg.GetSquaredLength() + dlm::Squared(PhantomXConst::kFemurLength) - dlm::Squared(PhantomXConst::kTibiaLength);
		const float arccos_lower = 2 * PhantomXConst::kFemurLength * femur_to_leg.GetLength();
		const float arccos_arg = arccos_upper / arccos_lower;

		const float fumur_joint_angle = std::acos(arccos_arg) + std::atan2(femur_to_leg_z, femur_to_leg_x);


		(*joint_state).joint_angle[1] = fumur_joint_angle;
	}

	// tibia joint�̌v�Z
	{
		const designlab::Vector3 femur_to_tibia = designlab::Vector3{
			PhantomXConst::kFemurLength * std::cos((*joint_state).joint_angle[0].value()) * std::cos((*joint_state).joint_angle[1].value()),
			PhantomXConst::kFemurLength * std::sin((*joint_state).joint_angle[0].value()) * std::cos((*joint_state).joint_angle[1].value()),
			PhantomXConst::kFemurLength * std::sin((*joint_state).joint_angle[1].value())
		};

		designlab::Vector3 tibia_joint_pos = (*joint_state).local_joint_position[1].value() + femur_to_tibia;

		(*joint_state).local_joint_position[2] = tibia_joint_pos;
	}


	// tibia angle�̌v�Z
	{
		float tibia_angle = std::atan2(
			(femur_to_leg_z - PhantomXConst::kFemurLength * std::sin((*joint_state).joint_angle[1].value())),
			(femur_to_leg_x - PhantomXConst::kFemurLength * std::cos((*joint_state).joint_angle[1].value()))
		) - (*joint_state).joint_angle[1].value();

		(*joint_state).joint_angle[2] = tibia_angle;
	}
}