#include "phantomx_state_calculator.h"

#include <cmath>

#include "designlab_line.h"


void PhantomXStateCalclator::init()
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


bool PhantomXStateCalclator::calculateAllJointState(const SNode& node, SHexapodJointState* const joint_state) const
{
	//�t�^���w�̎���Reference/Hexapod�̉摜���Q�Ƃ��Ă�������

	//���ʂ̏�����
	const int kLinkNum = 4;
	const int kJointNum = kLinkNum - 1;

	(*joint_state).local_joint_position.clear();
	(*joint_state).global_joint_position.clear();
	(*joint_state).joint_angle.clear();

	(*joint_state).local_joint_position.resize(HexapodConst::LEG_NUM);
	(*joint_state).global_joint_position.resize(HexapodConst::LEG_NUM);
	(*joint_state).joint_angle.resize(HexapodConst::LEG_NUM);

	for (int i = 0; i < HexapodConst::LEG_NUM; i++)
	{
		(*joint_state).local_joint_position[i].resize(kLinkNum);
		(*joint_state).global_joint_position[i].resize(kLinkNum);
		(*joint_state).joint_angle[i].resize(kJointNum);
	}

	//�v�Z���s���D
	for (int i = 0; i < HexapodConst::LEG_NUM; i++)
	{
		const float coxa_joint_angle = std::atan2(node.leg_pos[i].y, node.leg_pos[i].x);

		const dl_vec::SVector local_femur_joint_pos = dl_vec::SVector{ HexapodConst::PHANTOMX_COXA_LENGTH * std::cos(coxa_joint_angle), HexapodConst::PHANTOMX_COXA_LENGTH * std::sin(coxa_joint_angle), 0 };


		const float L = (node.leg_pos[i].projectedXY() - local_femur_joint_pos.projectedXY()).length();				//�r�悩����֐߂܂ł̒����D

		const float leg_to_fumur_len = std::sqrt(dl_math::squared(L) + dl_math::squared(node.leg_pos[i].z));

		const float s1 = dl_math::squared(leg_to_fumur_len) + dl_math::squared(HexapodConst::PHANTOMX_TIBIA_LENGTH) - dl_math::squared(HexapodConst::PHANTOMX_FEMUR_LENGTH);
		const float s2 = 2 * HexapodConst::PHANTOMX_TIBIA_LENGTH * leg_to_fumur_len;

		const float end_angle = -(std::acos(s1 / s2) + std::atan(-node.leg_pos[i].z / L));

		const dl_vec::SVector local_tibia_joint_pos = node.leg_pos[i] -
			dl_vec::SVector {HexapodConst::PHANTOMX_TIBIA_LENGTH* std::cos(coxa_joint_angle)* std::cos(end_angle),
			HexapodConst::PHANTOMX_TIBIA_LENGTH* std::sin(coxa_joint_angle)* std::cos(end_angle),
			HexapodConst::PHANTOMX_TIBIA_LENGTH* std::sin(end_angle)};


		const float fumur_joint_angle = std::atan2f(
			local_tibia_joint_pos.z - local_femur_joint_pos.z,
			local_tibia_joint_pos.projectedXY().length() - local_femur_joint_pos.projectedXY().length()
		);

		const float tibia_joint_angle = std::atan2f(
			node.leg_pos[i].z - local_tibia_joint_pos.z,
			node.leg_pos[i].projectedXY().length() - local_tibia_joint_pos.projectedXY().length()
		);


		(*joint_state).local_joint_position[i][0] = { 0,0,0 };
		(*joint_state).local_joint_position[i][1] = local_femur_joint_pos;
		(*joint_state).local_joint_position[i][2] = local_tibia_joint_pos;
		(*joint_state).local_joint_position[i][3] = node.leg_pos[i];

		(*joint_state).global_joint_position[i][0] = node.global_center_of_mass + dl_vec::rotVector(getLocalLegBasePosition(i), node.rot);
		(*joint_state).global_joint_position[i][1] = node.global_center_of_mass + dl_vec::rotVector(local_femur_joint_pos, node.rot);
		(*joint_state).global_joint_position[i][2] = node.global_center_of_mass + dl_vec::rotVector(local_tibia_joint_pos, node.rot);
		(*joint_state).global_joint_position[i][3] = node.global_center_of_mass + dl_vec::rotVector(node.leg_pos[i], node.rot);

		(*joint_state).joint_angle[i][0] = coxa_joint_angle;
		(*joint_state).joint_angle[i][1] = fumur_joint_angle;
		(*joint_state).joint_angle[i][2] = tibia_joint_angle;
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


	// coxa�֐߂͈͓̔��ɑ��݂��Ă��邩���m�F����
	float coxa_joint_angle = 0;

	if (x_pos == 0 && y_pos == 0)
	{
		coxa_joint_angle = HexapodConst::PHANTOMX_COXA_DEFAULT_ANGLE[leg_index];
	}
	else
	{
		coxa_joint_angle = std::atan2(y_pos, x_pos);
	}

	if (HexapodConst::PHANTOMX_COXA_ANGLE_MIN + HexapodConst::PHANTOMX_COXA_DEFAULT_ANGLE[leg_index] > coxa_joint_angle)
	{
		return false;
	}
	if (HexapodConst::PHANTOMX_COXA_ANGLE_MAX + HexapodConst::PHANTOMX_COXA_DEFAULT_ANGLE[leg_index] < coxa_joint_angle)
	{
		return false;
	}


	//�O�p�`�̕ӂ̒����̊֌W����C�L���ȋ��������m�F����
	dl_vec::SVector femur_joint_pos = dl_vec::SVector{
		std::cos(coxa_joint_angle) * HexapodConst::PHANTOMX_COXA_LENGTH,	std::sin(coxa_joint_angle) * HexapodConst::PHANTOMX_COXA_LENGTH, 0
	};

	const float a = HexapodConst::PHANTOMX_FEMUR_LENGTH;
	const float b = HexapodConst::PHANTOMX_TIBIA_LENGTH;
	const float c = (leg_pos - femur_joint_pos).length();

	if (a + b < c) { return false; }
	if (b + c < a) { return false; }
	if (c + a < b) { return false; }


	//�����܂Ń`�F�b�N���āC����������x�i������C�t�^���w�Ń`�F�b�N����
	{
		const float L = (leg_pos.projectedXY() - femur_joint_pos.projectedXY()).length();				//�r�悩����֐߂܂ł̒����D

		const float leg_to_fumur_len = std::sqrt(dl_math::squared(L) + dl_math::squared(leg_pos.z));

		const float s1 = dl_math::squared(leg_to_fumur_len) + dl_math::squared(HexapodConst::PHANTOMX_TIBIA_LENGTH) - dl_math::squared(HexapodConst::PHANTOMX_FEMUR_LENGTH);
		const float s2 = 2 * HexapodConst::PHANTOMX_TIBIA_LENGTH * leg_to_fumur_len;

		const float end_angle = -(std::acos(s1 / s2) + std::atan(-leg_pos.z / L));

		const dl_vec::SVector local_tibia_joint_pos = leg_pos -
			dl_vec::SVector {HexapodConst::PHANTOMX_TIBIA_LENGTH* std::cos(coxa_joint_angle)* std::cos(end_angle),
			HexapodConst::PHANTOMX_TIBIA_LENGTH* std::sin(coxa_joint_angle)* std::cos(end_angle),
			HexapodConst::PHANTOMX_TIBIA_LENGTH* std::sin(end_angle)};


		const float fumur_joint_angle = std::atan2f(
			local_tibia_joint_pos.z - femur_joint_pos.z,
			local_tibia_joint_pos.projectedXY().length() - femur_joint_pos.projectedXY().length()
		);

		const float tibia_joint_angle = std::atan2f(
			leg_pos.z - local_tibia_joint_pos.z,
			leg_pos.projectedXY().length() - local_tibia_joint_pos.projectedXY().length()
		);


		if (fumur_joint_angle < HexapodConst::PHANTOMX_FEMUR_ANGLE_MIN || HexapodConst::PHANTOMX_FEMUR_ANGLE_MAX < fumur_joint_angle) { return false; }
		if (tibia_joint_angle < HexapodConst::PHANTOMX_TIBIA_ANGLE_MIN || HexapodConst::PHANTOMX_TIBIA_ANGLE_MAX < tibia_joint_angle) { return false; }
	}

	return true;
}
