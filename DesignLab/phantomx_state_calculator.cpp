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
	return m_is_able_leg_pos[leg_index][getLegPosIndex(leg_pos.x)][getLegPosIndex(leg_pos.y)][getLegPosIndex(leg_pos.z)];
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


	SHexapodJointState joint_state;

	calculateLocalJointState(leg_index, leg_pos, &joint_state);


	// coxa�֐߂͈͓̔��ɑ��݂��Ă��邩���m�F����
	if (HexapodConst::PHANTOMX_COXA_ANGLE_MIN + HexapodConst::PHANTOMX_COXA_DEFAULT_ANGLE[leg_index] > joint_state.joint_angle[0]) { return false; }

	if (HexapodConst::PHANTOMX_COXA_ANGLE_MAX + HexapodConst::PHANTOMX_COXA_DEFAULT_ANGLE[leg_index] < joint_state.joint_angle[0]) { return false; }

	// femur�֐߂͈͓̔��ɑ��݂��Ă��邩���m�F����
	if (joint_state.joint_angle[1] < HexapodConst::PHANTOMX_FEMUR_ANGLE_MIN || HexapodConst::PHANTOMX_FEMUR_ANGLE_MAX < joint_state.joint_angle[1]) { return false; }

	// tibia�֐߂͈͓̔��ɑ��݂��Ă��邩���m�F����
	if (joint_state.joint_angle[2] < HexapodConst::PHANTOMX_TIBIA_ANGLE_MIN || HexapodConst::PHANTOMX_TIBIA_ANGLE_MAX < joint_state.joint_angle[2]) { return false; }


	// �����N�̒������m�F����
	if (dl_math::isEqual((joint_state.local_joint_position[0] - joint_state.local_joint_position[1]).length(), HexapodConst::PHANTOMX_COXA_LENGTH)) { return false; }

	//if (dl_math::isEqual((joint_state.local_joint_position[1] - joint_state.local_joint_position[2]).length(), HexapodConst::PHANTOMX_FEMUR_LENGTH)) { return false; }

	//if (dl_math::isEqual((joint_state.local_joint_position[2] - joint_state.local_joint_position[3]).length(), HexapodConst::PHANTOMX_TIBIA_LENGTH)) { return false; }


	//// coxa�֐߂͈͓̔��ɑ��݂��Ă��邩���m�F����
	//float coxa_joint_angle = 0;

	//if (x_pos == 0 && y_pos == 0)
	//{
	//	coxa_joint_angle = HexapodConst::PHANTOMX_COXA_DEFAULT_ANGLE[leg_index];
	//}
	//else
	//{
	//	coxa_joint_angle = std::atan2(y_pos, x_pos);
	//}

	//if (HexapodConst::PHANTOMX_COXA_ANGLE_MIN + HexapodConst::PHANTOMX_COXA_DEFAULT_ANGLE[leg_index] > coxa_joint_angle)
	//{
	//	return false;
	//}
	//if (HexapodConst::PHANTOMX_COXA_ANGLE_MAX + HexapodConst::PHANTOMX_COXA_DEFAULT_ANGLE[leg_index] < coxa_joint_angle)
	//{
	//	return false;
	//}


	////�O�p�`�̕ӂ̒����̊֌W����C�L���ȋ��������m�F����
	//dl_vec::SVector femur_joint_pos = dl_vec::SVector{
	//	std::cos(coxa_joint_angle) * HexapodConst::PHANTOMX_COXA_LENGTH,	std::sin(coxa_joint_angle) * HexapodConst::PHANTOMX_COXA_LENGTH, 0
	//};

	//const float a = HexapodConst::PHANTOMX_FEMUR_LENGTH;
	//const float b = HexapodConst::PHANTOMX_TIBIA_LENGTH;
	//const float c = (leg_pos - femur_joint_pos).length();

	//if (a + b < c) { return false; }
	//if (b + c < a) { return false; }
	//if (c + a < b) { return false; }


	////�����܂Ń`�F�b�N���āC����������x�i������C�t�^���w�Ń`�F�b�N����
	//{
	//	const dl_vec::SVector local_femur_joint_pos = dl_vec::SVector{ HexapodConst::PHANTOMX_COXA_LENGTH * std::cos(coxa_joint_angle), HexapodConst::PHANTOMX_COXA_LENGTH * std::sin(coxa_joint_angle), 0 };

	//	const float L = (leg_pos.projectedXY() - femur_joint_pos.projectedXY()).length();				//�r�悩����֐߂܂ł̒����D

	//	const float leg_to_fumur_len = std::sqrt(dl_math::squared(L) + dl_math::squared(leg_pos.z));

	//	const float s1 = dl_math::squared(leg_to_fumur_len) + dl_math::squared(HexapodConst::PHANTOMX_TIBIA_LENGTH) - dl_math::squared(HexapodConst::PHANTOMX_FEMUR_LENGTH);
	//	const float s2 = 2 * HexapodConst::PHANTOMX_TIBIA_LENGTH * leg_to_fumur_len;

	//	float end_angle = -(std::acos(s1 / s2) + std::atan(-leg_pos.z / L));

	//	dl_vec::SVector local_tibia_joint_pos = leg_pos -
	//		dl_vec::SVector {HexapodConst::PHANTOMX_TIBIA_LENGTH* std::cos(coxa_joint_angle)* std::cos(end_angle),
	//		HexapodConst::PHANTOMX_TIBIA_LENGTH* std::sin(coxa_joint_angle)* std::cos(end_angle),
	//		HexapodConst::PHANTOMX_TIBIA_LENGTH* std::sin(end_angle)};

	//	if (!dl_math::isEqual((local_femur_joint_pos - local_tibia_joint_pos).length(), HexapodConst::PHANTOMX_FEMUR_LENGTH))
	//	{
	//		end_angle = -(-std::acos(s1 / s2) + std::atan(-leg_pos.z / L));

	//		local_tibia_joint_pos = leg_pos -
	//			dl_vec::SVector {HexapodConst::PHANTOMX_TIBIA_LENGTH* std::cos(coxa_joint_angle)* std::cos(end_angle),
	//			HexapodConst::PHANTOMX_TIBIA_LENGTH* std::sin(coxa_joint_angle)* std::cos(end_angle),
	//			HexapodConst::PHANTOMX_TIBIA_LENGTH* std::sin(end_angle)};
	//	}


	//	const float fumur_joint_angle = std::atan2f(
	//		local_tibia_joint_pos.z - femur_joint_pos.z,
	//		local_tibia_joint_pos.projectedXY().length() - femur_joint_pos.projectedXY().length()
	//	);

	//	const float tibia_joint_angle = std::atan2f(
	//		leg_pos.z - local_tibia_joint_pos.z,
	//		leg_pos.projectedXY().length() - local_tibia_joint_pos.projectedXY().length()
	//	);


	//	if (fumur_joint_angle < HexapodConst::PHANTOMX_FEMUR_ANGLE_MIN || HexapodConst::PHANTOMX_FEMUR_ANGLE_MAX < fumur_joint_angle) { return false; }
	//	if (tibia_joint_angle < HexapodConst::PHANTOMX_TIBIA_ANGLE_MIN || HexapodConst::PHANTOMX_TIBIA_ANGLE_MAX < tibia_joint_angle) { return false; }
	//}

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


	// tibia joint�̌v�Z
	const float L = (leg_pos.projectedXY() - femur_joint_pos.projectedXY()).length();				//�r�悩����֐߂܂ł̒����D

	const float leg_to_fumur_len = std::sqrt(dl_math::squared(L) + dl_math::squared(leg_pos.z));

	const float s1 = dl_math::squared(leg_to_fumur_len) + dl_math::squared(HexapodConst::PHANTOMX_TIBIA_LENGTH) - dl_math::squared(HexapodConst::PHANTOMX_FEMUR_LENGTH);
	const float s2 = 2 * HexapodConst::PHANTOMX_TIBIA_LENGTH * leg_to_fumur_len;

	float end_angle = -(std::acos(s1 / s2) + std::atan(-leg_pos.z / L));

	dl_vec::SVector tibia_joint_pos = leg_pos -
		dl_vec::SVector {HexapodConst::PHANTOMX_TIBIA_LENGTH* std::cos(coxa_joint_angle)* std::cos(end_angle),
		HexapodConst::PHANTOMX_TIBIA_LENGTH* std::sin(coxa_joint_angle)* std::cos(end_angle),
		HexapodConst::PHANTOMX_TIBIA_LENGTH* std::sin(end_angle)};

	if (dl_math::isEqual((femur_joint_pos - tibia_joint_pos).length(), HexapodConst::PHANTOMX_FEMUR_LENGTH))
	{
		tibia_joint_pos = leg_pos -
			dl_vec::SVector {HexapodConst::PHANTOMX_TIBIA_LENGTH* std::cos(coxa_joint_angle)* std::cos(end_angle),
			HexapodConst::PHANTOMX_TIBIA_LENGTH* std::sin(coxa_joint_angle)* std::cos(end_angle),
			HexapodConst::PHANTOMX_TIBIA_LENGTH* std::sin(end_angle)};
	}

	(*joint_state).local_joint_position[2] = tibia_joint_pos;


	// femur angle�̌v�Z
	const float fumur_joint_angle = std::atan2f(
		femur_joint_pos.z - tibia_joint_pos.z,
		tibia_joint_pos.projectedXY().length() - femur_joint_pos.projectedXY().length()
	);

	(*joint_state).joint_angle[1] = fumur_joint_angle;


	// tibia angle�̌v�Z
	const float tibia_joint_angle = std::atan2f(
		tibia_joint_pos.z - leg_pos.z,
		leg_pos.projectedXY().length() - tibia_joint_pos.projectedXY().length()
	);

	(*joint_state).joint_angle[2] = tibia_joint_angle;


	// �r��̒ǉ�
	(*joint_state).local_joint_position[3] = leg_pos;
}
