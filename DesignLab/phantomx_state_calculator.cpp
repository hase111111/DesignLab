#include "phantomx_state_calculator.h"

#include <cmath>

#include "designlab_line.h"



PhantomXStateCalclator::PhantomXStateCalclator()
{
	//�r�̕t�����̈ʒu������������
	m_local_leg_base_pos[0] = dl_vec::SVector(HexapodConst::BODY_FRONT_LENGTH, -HexapodConst::BODY_FRONT_WIDTH, 0.0f);	// �r0 �E��
	m_local_leg_base_pos[1] = dl_vec::SVector(0.0f, -HexapodConst::BODY_CENTER_WIDTH, 0.0f);	// �r1 �E��
	m_local_leg_base_pos[2] = dl_vec::SVector(-HexapodConst::BODY_REAR_LENGTH, -HexapodConst::BODY_REAR_WIDTH, 0.0f);	// �r2 �E��
	m_local_leg_base_pos[3] = dl_vec::SVector(-HexapodConst::BODY_REAR_LENGTH, HexapodConst::BODY_REAR_WIDTH, 0.0f);	// �r3 ����
	m_local_leg_base_pos[4] = dl_vec::SVector(0.0f, HexapodConst::BODY_CENTER_WIDTH, 0.0f);	// �r4 ����
	m_local_leg_base_pos[5] = dl_vec::SVector(HexapodConst::BODY_FRONT_LENGTH, HexapodConst::BODY_FRONT_WIDTH, 0.0f);	// �r5 ����


	// m_is_able_leg_pos ������������D������4�d���[�v
	for (int i = 0; i < HexapodConst::LEG_NUM; i++)
	{
		for (int x = 0; x < LEG_POS_DIV_NUM; x++)
		{
			for (int y = 0; y < LEG_POS_DIV_NUM; y++)
			{
				for (int z = 0; z < LEG_POS_DIV_NUM; z++)
				{
					m_is_able_leg_pos[i][x][y][z] = initIsAbleLegPos(i, x, y, z);
				}
			}
		}
	}
}


bool PhantomXStateCalclator::calculateAllJointState(const SNode& node, SHexapodJointState joint_state[HexapodConst::LEG_NUM]) const
{
	//�t�^���w�̎���Reference/Hexapod�̉摜���Q�Ƃ��Ă�������


	//�v�Z���s���D
	for (int i = 0; i < HexapodConst::LEG_NUM; i++)
	{
		calculateLocalJointState(i, node.leg_pos[i], &joint_state[i]);

		joint_state[i].global_joint_position.clear();
		joint_state[i].global_joint_position.resize(4);

		joint_state[i].global_joint_position[0] = node.global_center_of_mass + dl_vec::rotVector(getLocalLegBasePosition(i), node.rot);
		joint_state[i].global_joint_position[1] = node.global_center_of_mass + dl_vec::rotVector(getLocalLegBasePosition(i) + joint_state[i].local_joint_position[1], node.rot);
		joint_state[i].global_joint_position[2] = node.global_center_of_mass + dl_vec::rotVector(getLocalLegBasePosition(i) + joint_state[i].local_joint_position[2], node.rot);
		joint_state[i].global_joint_position[3] = node.global_center_of_mass + dl_vec::rotVector(getLocalLegBasePosition(i) + node.leg_pos[i], node.rot);
	}

	return false;
}


dl_vec::SVector PhantomXStateCalclator::convertGlobalToLegPosition(const int leg_index, const dl_vec::SVector& leg_pos, const dl_vec::SVector& global_center_of_mass, const dl_vec::SRotator& robot_rot, const bool consider_rot) const
{
	if (consider_rot)
	{
		return dl_vec::rotVector(leg_pos - global_center_of_mass, robot_rot * -1) - getLocalLegBasePosition(leg_index);
	}
	else
	{
		return leg_pos - global_center_of_mass - getLocalLegBasePosition(leg_index);
	}
}


dl_vec::SVector PhantomXStateCalclator::getLocalLegPosition(const int leg_index, const dl_vec::SVector& leg_pos) const
{
	return leg_pos + getLocalLegBasePosition(leg_index);
}


dl_vec::SVector PhantomXStateCalclator::getGlobalLegBasePosition(const int leg_index, const dl_vec::SVector& global_center_of_mass, const dl_vec::SRotator& robot_rot, const bool consider_rot) const
{
	if (consider_rot) { return dl_vec::rotVector(getLocalLegBasePosition(leg_index), robot_rot) + global_center_of_mass; }
	else { return getLocalLegBasePosition(leg_index) + global_center_of_mass; }
}


dl_vec::SVector PhantomXStateCalclator::getGlobalLegPosition(const int leg_index, const dl_vec::SVector& leg_pos, const dl_vec::SVector& global_center_of_mass, const dl_vec::SRotator& robot_rot, const bool consider_rot) const
{
	if (consider_rot) { return dl_vec::rotVector(getLocalLegBasePosition(leg_index) + leg_pos, robot_rot) + global_center_of_mass; }
	else { return global_center_of_mass + getLocalLegBasePosition(leg_index) + leg_pos; }
}


bool PhantomXStateCalclator::isLegInRange(const int leg_index, const dl_vec::SVector& leg_pos) const
{
	//�͈͊O�Ȃ�false
	if (getLegPosIndex(leg_pos.x) < 0 || LEG_POS_DIV_NUM <= getLegPosIndex(leg_pos.x)) { return false; }
	if (getLegPosIndex(leg_pos.y) < 0 || LEG_POS_DIV_NUM <= getLegPosIndex(leg_pos.y)) { return false; }
	if (getLegPosIndex(leg_pos.z) < 0 || LEG_POS_DIV_NUM <= getLegPosIndex(leg_pos.z)) { return false; }

	if (m_is_able_leg_pos[leg_index][getLegPosIndex(leg_pos.x)][getLegPosIndex(leg_pos.y)][getLegPosIndex(leg_pos.z)])
	{
		const dl_vec::SVector2 leg_pos_xy = leg_pos.projectedXY();
		const dl_vec::SVector2 min_leg_pos_xy{HexapodConst::MOVABLE_LEG_RANGE_COS_MIN[leg_index], HexapodConst::MOVABLE_LEG_RANGE_SIN_MAX[leg_index]};
		const dl_vec::SVector2 max_leg_pos_xy{HexapodConst::MOVABLE_LEG_RANGE_COS_MAX[leg_index], HexapodConst::MOVABLE_LEG_RANGE_SIN_MIN[leg_index]};

		//�r�̊p�x���͈͓��ɂ��邩���ׂ�D�O�όv�Z�ŊԂɂ��邩���ׂ�
		if (min_leg_pos_xy.cross(leg_pos_xy) > 0.0f) { return false; }
		if (max_leg_pos_xy.cross(leg_pos_xy) < 0.0f) { return false; }

		return true;
	}

	return false;
}


bool PhantomXStateCalclator::isLegInterfering(const dl_vec::SVector leg_pos[HexapodConst::LEG_NUM]) const
{
	//�d�S�����_�Ƃ����C���W�n�ɂ����āC�r�̊��𒲂ׂ�D

	//�r�̊��𒲂ׂ�D
	dl_vec::SVector2 leg_pos_xy[HexapodConst::LEG_NUM];
	dl_vec::SVector2 joint_pos_xy[HexapodConst::LEG_NUM];

	for (int i = 0; i < HexapodConst::LEG_NUM; i++)
	{
		joint_pos_xy[i] = getLocalLegBasePosition(i).projectedXY();
		leg_pos_xy[i] = leg_pos[i].projectedXY() + joint_pos_xy[i];
	}

	//�ׂ̋r�Ƃ̊��𒲂ׂ�D
	for (int i = 0; i < HexapodConst::LEG_NUM; i++)
	{
		dl_vec::SLine2 line1(joint_pos_xy[i], leg_pos_xy[i]);
		dl_vec::SLine2 line2(joint_pos_xy[(i + 1) % HexapodConst::LEG_NUM], leg_pos_xy[(i + 1) % HexapodConst::LEG_NUM]);

		if (line1.hasIntersection(line2)) { return true; }
	}

	return false;
}


bool PhantomXStateCalclator::initIsAbleLegPos(const int leg_index, const int x, const int y, const int z) const
{
	float x_pos = (LEG_POS_MAX - LEG_POS_MIN) / LEG_POS_DIV_NUM * x + LEG_POS_MIN;
	float y_pos = (LEG_POS_MAX - LEG_POS_MIN) / LEG_POS_DIV_NUM * y + LEG_POS_MIN;
	float z_pos = (LEG_POS_MAX - LEG_POS_MIN) / LEG_POS_DIV_NUM * z + LEG_POS_MIN;

	dl_vec::SVector leg_pos{x_pos, y_pos, z_pos};		//�r��̈ʒu


	//�� x=0 y=0 �̎���false�ɂ���
	if (abs(x_pos) <= (LEG_POS_MAX - LEG_POS_MIN) / (LEG_POS_DIV_NUM - 1.0f) && abs(y_pos) <= (LEG_POS_MAX - LEG_POS_MIN) / (LEG_POS_DIV_NUM - 1.0f)) { return false; }

	if (leg_pos.projectedXY().length() < MIN_LEG_R) { return false; }

	// �֐߂̊p�x���v�Z����
	SHexapodJointState joint_state;

	calculateLocalJointState(leg_index, leg_pos, &joint_state);


	// coxa�֐߂͈͓̔��ɑ��݂��Ă��邩���m�F����
	const float kCoxaMargim = dl_math::convertDegToRad(5.0f);

	if (HexapodConst::PHANTOMX_COXA_ANGLE_MIN + HexapodConst::PHANTOMX_COXA_DEFAULT_ANGLE[leg_index] + kCoxaMargim > joint_state.joint_angle[0]) { return false; }

	if (HexapodConst::PHANTOMX_COXA_ANGLE_MAX + HexapodConst::PHANTOMX_COXA_DEFAULT_ANGLE[leg_index] - kCoxaMargim < joint_state.joint_angle[0]) { return false; }

	// femur�֐߂͈͓̔��ɑ��݂��Ă��邩���m�F����
	const float kFemurMargim = dl_math::convertDegToRad(5.0f);

	if (joint_state.joint_angle[1] < HexapodConst::PHANTOMX_FEMUR_ANGLE_MIN + kFemurMargim || HexapodConst::PHANTOMX_FEMUR_ANGLE_MAX - kFemurMargim < joint_state.joint_angle[1]) { return false; }

	// tibia�֐߂͈͓̔��ɑ��݂��Ă��邩���m�F����
	const float kTibiaMargim = dl_math::convertDegToRad(5.0f);

	if (joint_state.joint_angle[2] < HexapodConst::PHANTOMX_TIBIA_ANGLE_MIN + kTibiaMargim || HexapodConst::PHANTOMX_TIBIA_ANGLE_MAX - kTibiaMargim < joint_state.joint_angle[2]) { return false; }

	// �����N�̒������m�F����
	if (!dl_math::isEqual((joint_state.local_joint_position[0] - joint_state.local_joint_position[1]).length(), HexapodConst::PHANTOMX_COXA_LENGTH)) { return false; }

	if (!dl_math::isEqual((joint_state.local_joint_position[1] - joint_state.local_joint_position[2]).length(), HexapodConst::PHANTOMX_FEMUR_LENGTH)) { return false; }

	if (!dl_math::isEqual((joint_state.local_joint_position[2] - joint_state.local_joint_position[3]).length(), HexapodConst::PHANTOMX_TIBIA_LENGTH)) { return false; }

	return true;
}


void PhantomXStateCalclator::calculateLocalJointState(const int leg_index, const dl_vec::SVector& leg_pos, SHexapodJointState* joint_state) const
{
	const int kLinkNum = 4;
	const int kJointNum = kLinkNum - 1;

	(*joint_state).local_joint_position.clear();
	(*joint_state).joint_angle.clear();

	(*joint_state).local_joint_position.resize(kLinkNum);
	(*joint_state).joint_angle.resize(kJointNum);


	// coxa joint�̌v�Z
	(*joint_state).local_joint_position[0] = dl_vec::SVector{ 0, 0, 0 };


	// coxa angle�̌v�Z
	float coxa_joint_angle = std::atan2f(leg_pos.y, leg_pos.x);

	if (leg_pos.y == 0 && leg_pos.x == 0) { coxa_joint_angle = HexapodConst::PHANTOMX_COXA_DEFAULT_ANGLE[leg_index]; }

	(*joint_state).joint_angle[0] = coxa_joint_angle;


	// femur joint�̌v�Z
	const dl_vec::SVector femur_joint_pos = dl_vec::SVector{ HexapodConst::PHANTOMX_COXA_LENGTH * std::cos(coxa_joint_angle), HexapodConst::PHANTOMX_COXA_LENGTH * std::sin(coxa_joint_angle), 0 };

	(*joint_state).local_joint_position[1] = femur_joint_pos;


	if ((leg_pos - femur_joint_pos).length() > HexapodConst::PHANTOMX_FEMUR_LENGTH + HexapodConst::PHANTOMX_TIBIA_LENGTH) { return; }


	// tibia joint / femur angle �̌v�Z
	const float leg_to_f_x = leg_pos.projectedXY().length() - femur_joint_pos.projectedXY().length();				//�r�悩����֐߂܂ł̒����D
	const float leg_to_f_y = leg_pos.z - femur_joint_pos.z;

	const float arccos_arg = (dl_math::squared(leg_to_f_x) + dl_math::squared(leg_to_f_y) + dl_math::squared(HexapodConst::PHANTOMX_FEMUR_LENGTH) - dl_math::squared(HexapodConst::PHANTOMX_TIBIA_LENGTH))
		/ (2 * HexapodConst::PHANTOMX_FEMUR_LENGTH * std::sqrt(dl_math::squared(leg_to_f_x) + dl_math::squared(leg_to_f_y)));

	float fumur_joint_angle1 = std::acos(arccos_arg) + std::atan2(leg_to_f_y, leg_to_f_x);
	float fumur_joint_angle2 = -std::acos(arccos_arg) + std::atan2(leg_to_f_y, leg_to_f_x);

	dl_vec::SVector tibia_joint_pos1 = femur_joint_pos +
		dl_vec::SVector { HexapodConst::PHANTOMX_FEMUR_LENGTH* std::cos(coxa_joint_angle)* std::cos(fumur_joint_angle1),
		HexapodConst::PHANTOMX_FEMUR_LENGTH* std::sin(coxa_joint_angle)* std::cos(fumur_joint_angle1),
		HexapodConst::PHANTOMX_FEMUR_LENGTH* std::sin(fumur_joint_angle1)
	};

	dl_vec::SVector tibia_joint_pos2 = femur_joint_pos +
		dl_vec::SVector { HexapodConst::PHANTOMX_FEMUR_LENGTH* std::cos(coxa_joint_angle)* std::cos(fumur_joint_angle2),
		HexapodConst::PHANTOMX_FEMUR_LENGTH* std::sin(coxa_joint_angle)* std::cos(fumur_joint_angle2),
		HexapodConst::PHANTOMX_FEMUR_LENGTH* std::sin(fumur_joint_angle2)
	};

	//if (tibia_joint_pos1.projectedXY().lengthSquare() < tibia_joint_pos2.projectedXY().lengthSquare())
	//{  ������͊Ԉ���Ă�
	if (false)
	{
		(*joint_state).local_joint_position[2] = tibia_joint_pos2;

		(*joint_state).joint_angle[1] = fumur_joint_angle2;
	}
	else
	{
		(*joint_state).local_joint_position[2] = tibia_joint_pos1;

		(*joint_state).joint_angle[1] = fumur_joint_angle1;
	}

	(*joint_state).joint_angle[2] = std::atan2((leg_to_f_y - HexapodConst::PHANTOMX_FEMUR_LENGTH * std::sin((*joint_state).joint_angle[1])),
		(leg_to_f_x - HexapodConst::PHANTOMX_FEMUR_LENGTH * std::cos((*joint_state).joint_angle[1]))) - (*joint_state).joint_angle[1];


	// �r��̒ǉ�
	(*joint_state).local_joint_position[3] = leg_pos;
}
