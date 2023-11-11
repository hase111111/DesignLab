#include "phantomx_mk2.h"

#include <cmath>

#include "cassert_define.h"
#include "designlab_line_segment2.h"
#include "designlab_math_util.h"
#include "leg_state.h"
#include "phantomx_mk2_const.h"


namespace dl = ::designlab;
namespace dlm = ::designlab::math_util;
namespace dllf = ::designlab::leg_func;


PhantomXMkII::PhantomXMkII() :
	free_leg_pos_leg_coordinate_({ {
		{170 * cos(PhantomXMkIIConst::kCoxaDefaultAngle[0]), 170 * sin(PhantomXMkIIConst::kCoxaDefaultAngle[0]), kFreeLegHeight},
		{170 * cos(PhantomXMkIIConst::kCoxaDefaultAngle[1]), 170 * sin(PhantomXMkIIConst::kCoxaDefaultAngle[1]), kFreeLegHeight},
		{170 * cos(PhantomXMkIIConst::kCoxaDefaultAngle[2]), 170 * sin(PhantomXMkIIConst::kCoxaDefaultAngle[2]), kFreeLegHeight},
		{170 * cos(PhantomXMkIIConst::kCoxaDefaultAngle[3]), 170 * sin(PhantomXMkIIConst::kCoxaDefaultAngle[3]), kFreeLegHeight},
		{170 * cos(PhantomXMkIIConst::kCoxaDefaultAngle[4]), 170 * sin(PhantomXMkIIConst::kCoxaDefaultAngle[4]), kFreeLegHeight},
		{170 * cos(PhantomXMkIIConst::kCoxaDefaultAngle[5]), 170 * sin(PhantomXMkIIConst::kCoxaDefaultAngle[5]), kFreeLegHeight},
	} }),
	leg_base_pos_robot_coordinate_({ {
		{ PhantomXMkIIConst::kCoxaBaseOffsetX, -PhantomXMkIIConst::kCoxaBaseOffsetY, PhantomXMkIIConst::kCoxaBaseOffsetZ },	// �r0 �E��
		{ 0.0f, -PhantomXMkIIConst::kCenterCoxaBaseOffsetY, PhantomXMkIIConst::kCoxaBaseOffsetZ },						// �r1 �E��
		{ -PhantomXMkIIConst::kCoxaBaseOffsetX, -PhantomXMkIIConst::kCoxaBaseOffsetY, PhantomXMkIIConst::kCoxaBaseOffsetZ },// �r2 �E��
		{ -PhantomXMkIIConst::kCoxaBaseOffsetX, PhantomXMkIIConst::kCoxaBaseOffsetY, PhantomXMkIIConst::kCoxaBaseOffsetZ },	// �r3 ����
		{ 0.0f, PhantomXMkIIConst::kCenterCoxaBaseOffsetY, PhantomXMkIIConst::kCoxaBaseOffsetZ },						// �r4 ����
		{ PhantomXMkIIConst::kCoxaBaseOffsetX, PhantomXMkIIConst::kCoxaBaseOffsetY, PhantomXMkIIConst::kCoxaBaseOffsetZ },	// �r5 ����
	}}),
	kMaxLegR(InitMaxLegR()),
	kMinLegPosXY(InitMinLegPosXY()),
	kMaxLegPosXY(InitMaxLegPosXY())
{
}


std::array<HexapodJointState, HexapodConst::kLegNum> PhantomXMkII::CalculateAllJointState(const RobotStateNode& node) const noexcept
{
	std::array<HexapodJointState, HexapodConst::kLegNum> joint_state;

	//�v�Z���s���D
	for (int i = 0; i < HexapodConst::kLegNum; i++)
	{
		joint_state[i] = CalculateJointState(i, node.leg_pos[i]);
	}

	return joint_state;
}

HexapodJointState PhantomXMkII::CalculateJointState(const int leg_index, const dl::Vector3& leg_pos) const noexcept
{
	//leg_index�� 0�`5 �ł���D
	assert(0 <= leg_index);
	assert(leg_index < HexapodConst::kLegNum);

	//�e�p�����[�^������������
	HexapodJointState res;

	const int kJointNum = 4;
	const int kJointAngleNum = kJointNum - 1;

	res.joint_pos_leg_coordinate.clear();
	res.joint_angle.clear();

	res.joint_pos_leg_coordinate.resize(kJointNum);
	res.joint_angle.resize(kJointAngleNum);


	// coxa joint�̌v�Z
	res.joint_pos_leg_coordinate[0] = designlab::Vector3{ 0, 0, 0 };	//�r���W�n�ł�coxa joint�͌��_�ɂ���D

	// �r��̒ǉ�
	res.joint_pos_leg_coordinate[3] = leg_pos;

	// coxa angle�̌v�Z
	{
		float coxa_joint_angle = std::atan2f(leg_pos.y, leg_pos.x);

		if (leg_pos.y == 0 && leg_pos.x == 0) { coxa_joint_angle = PhantomXMkIIConst::kCoxaDefaultAngle[leg_index]; }

		if (!PhantomXMkIIConst::IsVaildCoxaAngle(leg_index, coxa_joint_angle))
		{
			//�͈͊O�Ȃ�΁C180�x��]���������ɔ͈͓��ɂ��邩�𒲂ׂ�D
			if (PhantomXMkIIConst::IsVaildCoxaAngle(leg_index, coxa_joint_angle + dlm::kFloatPi))
			{
				coxa_joint_angle += dlm::kFloatPi;
			}
			else if (PhantomXMkIIConst::IsVaildCoxaAngle(leg_index, coxa_joint_angle - dlm::kFloatPi))
			{
				coxa_joint_angle -= dlm::kFloatPi;
			}
		}

		res.joint_angle[0] = coxa_joint_angle;
	}

	// femur joint�̌v�Z
	{
		const dl::Vector3 femur_joint_pos = dl::Vector3{
			PhantomXMkIIConst::kCoxaLength * std::cos(res.joint_angle[0]),
			PhantomXMkIIConst::kCoxaLength * std::sin(res.joint_angle[0]),
			0
		};

		res.joint_pos_leg_coordinate[1] = femur_joint_pos;

		if ( ! dlm::CanMakeTriangle((leg_pos - femur_joint_pos).GetLength(), PhantomXMkIIConst::kFemurLength, PhantomXMkIIConst::kTibiaLength))
		{
			// ���������r�悪�r�̕t��������͂��Ȃ��ꍇ�C��ԋ߂��ʒu�܂ŋr��L�΂��D

			const float angle_ft = std::atan2(leg_pos.z - femur_joint_pos.z, (leg_pos.ProjectedXY() - femur_joint_pos.ProjectedXY()).GetLength());

			// angle_ft�̈ʑ���180�x��]����D-180�x�`180�x�͈̔͂ɂ���D
			float angle_ft_phase = angle_ft + dlm::kFloatPi;
			angle_ft_phase = angle_ft_phase > dlm::kFloatPi ? angle_ft_phase - dlm::kFloatPi : angle_ft_phase;

			const dl::Vector3 candidate_leg_pos = femur_joint_pos + dl::Vector3{
				(PhantomXMkIIConst::kFemurLength + PhantomXMkIIConst::kTibiaLength) * std::cos(res.joint_angle[0]) * std::cos(angle_ft),
				(PhantomXMkIIConst::kFemurLength + PhantomXMkIIConst::kTibiaLength) * std::sin(res.joint_angle[0]) * std::cos(angle_ft) ,
				(PhantomXMkIIConst::kFemurLength + PhantomXMkIIConst::kTibiaLength) * std::sin(angle_ft)
			};

			const dl::Vector3 candidate_leg_pos_phase = femur_joint_pos + dl::Vector3{
				(PhantomXMkIIConst::kFemurLength * std::cos(angle_ft_phase) + PhantomXMkIIConst::kTibiaLength * std::cos(angle_ft)) * std::cos(res.joint_angle[0]),
				(PhantomXMkIIConst::kFemurLength * std::cos(angle_ft_phase) + PhantomXMkIIConst::kTibiaLength * std::cos(angle_ft)) * std::sin(res.joint_angle[0]),
				PhantomXMkIIConst::kFemurLength * std::sin(angle_ft_phase) + PhantomXMkIIConst::kTibiaLength * std::sin(angle_ft)
			};

			const float distance = (leg_pos - candidate_leg_pos).GetLength();
			const float distance_phase = (leg_pos - candidate_leg_pos_phase).GetLength();

			float angle_f = 0, angle_t = 0;

			if (distance < distance_phase)
			{
				angle_f = angle_ft;
				angle_t = 0;
			}
			else
			{
				angle_f = angle_ft_phase;
				angle_t = -dlm::kFloatPi;
			}

			res.joint_angle[1] = angle_f;
			res.joint_angle[2] = angle_t;

			res.joint_pos_leg_coordinate[2] = femur_joint_pos + designlab::Vector3{
				PhantomXMkIIConst::kFemurLength * std::cos(angle_f) * std::cos(res.joint_angle[0]),
				PhantomXMkIIConst::kFemurLength * std::cos(angle_f) * std::sin(res.joint_angle[0]),
				PhantomXMkIIConst::kFemurLength * std::sin(angle_f)
			};

			res.joint_pos_leg_coordinate[3] = res.joint_pos_leg_coordinate[2] + designlab::Vector3{
				PhantomXMkIIConst::kTibiaLength * std::cos(angle_f + angle_t) * std::cos(res.joint_angle[0]),
				PhantomXMkIIConst::kTibiaLength * std::cos(angle_f + angle_t) * std::sin(res.joint_angle[0]),
				PhantomXMkIIConst::kTibiaLength * std::sin(angle_f + angle_t)
			};

			res.is_in_range = false;	//�͈͊O�ł��邱�Ƃ������D

			return res;
		}
	}


	// femur angle �̌v�Z
	const designlab::Vector3 femur_to_leg = leg_pos - res.joint_pos_leg_coordinate[1];		//�r�悩����֐߂܂ł̒����D
	const float femur_to_leg_x = femur_to_leg.ProjectedXY().GetLength() * 																//�r��֌�����������x�̐������ɂ�����W�n�ɒu�������� 			
		((leg_pos.ProjectedXY().GetSquaredLength() > res.joint_pos_leg_coordinate[1].ProjectedXY().GetSquaredLength()) ? 1.f : -1.f);	//�r�悪���֐߂����߂��ꍇ�͐��̕����ɂ���D
	const float femur_to_leg_z = femur_to_leg.z;

	{
		const float arccos_upper = femur_to_leg.GetSquaredLength() + dlm::Squared(PhantomXMkIIConst::kFemurLength) - dlm::Squared(PhantomXMkIIConst::kTibiaLength);
		const float arccos_lower = 2 * PhantomXMkIIConst::kFemurLength * femur_to_leg.GetLength();
		const float arccos_arg = arccos_upper / arccos_lower;

		const float fumur_joint_angle = std::acos(arccos_arg) + std::atan2(femur_to_leg_z, femur_to_leg_x);

		res.joint_angle[1] = fumur_joint_angle;
	}

	// tibia joint�̌v�Z
	{
		const designlab::Vector3 femur_to_tibia = designlab::Vector3{
			PhantomXMkIIConst::kFemurLength * std::cos(res.joint_angle[0]) * std::cos(res.joint_angle[1]),
			PhantomXMkIIConst::kFemurLength * std::sin(res.joint_angle[0]) * std::cos(res.joint_angle[1]),
			PhantomXMkIIConst::kFemurLength * std::sin(res.joint_angle[1])
		};

		const designlab::Vector3 tibia_joint_pos = res.joint_pos_leg_coordinate[1] + femur_to_tibia;

		res.joint_pos_leg_coordinate[2] = tibia_joint_pos;
	}


	// tibia angle�̌v�Z
	{
		const float tibia_angle = std::atan2(
			(femur_to_leg_z - PhantomXMkIIConst::kFemurLength * std::sin(res.joint_angle[1])),
			(femur_to_leg_x - PhantomXMkIIConst::kFemurLength * std::cos(res.joint_angle[1]))
		) - res.joint_angle[1];

		res.joint_angle[2] = tibia_angle;
	}

	res.is_in_range = true;

	return res;
}

bool PhantomXMkII::IsVaildAllJointState(const RobotStateNode& node, const std::array<HexapodJointState, HexapodConst::kLegNum>& joint_state) const noexcept
{
	for (int i = 0; i < HexapodConst::kLegNum; i++)
	{
		if (!IsVaildJointState(i, node.leg_pos[i], joint_state[i]))
		{
			return false;
		}
	}

	return true;
}

bool PhantomXMkII::IsVaildJointState(const int leg_index, const dl::Vector3& leg_pos, const HexapodJointState& joint_state) const noexcept
{
	assert(joint_state.joint_pos_leg_coordinate.size() == 4);
	assert(joint_state.joint_angle.size() == 3);

	// coxa�֐߂͈͓̔��ɑ��݂��Ă��邩���m�F����
	if (!PhantomXMkIIConst::IsVaildCoxaAngle(leg_index, joint_state.joint_angle[0])) { return false; }

	// femur�֐߂͈͓̔��ɑ��݂��Ă��邩���m�F����
	if (!PhantomXMkIIConst::IsVaildFemurAngle(joint_state.joint_angle[1])) { return false; }

	// tibia�֐߂͈͓̔��ɑ��݂��Ă��邩���m�F����
	if (!PhantomXMkIIConst::IsVaildTibiaAngle(joint_state.joint_angle[2])) { return false; }

	// �����N�̒������m�F����
	if (!dlm::IsEqual((joint_state.joint_pos_leg_coordinate[0] - joint_state.joint_pos_leg_coordinate[1]).GetLength(), PhantomXMkIIConst::kCoxaLength))
	{
		return false;
	}

	if (!dlm::IsEqual((joint_state.joint_pos_leg_coordinate[1] - joint_state.joint_pos_leg_coordinate[2]).GetLength(), PhantomXMkIIConst::kFemurLength))
	{
		return false;
	}

	if (!dlm::IsEqual((joint_state.joint_pos_leg_coordinate[2] - joint_state.joint_pos_leg_coordinate[3]).GetLength(), PhantomXMkIIConst::kTibiaLength))
	{
		return false;
	}

	// �r�ʒu�Ƌr����W����v���Ă��邩���m�F����
	if (joint_state.joint_pos_leg_coordinate[3] != leg_pos) { return false; }

	return true;
}


dl::Vector3 PhantomXMkII::ConvertGlobalToLegCoordinate(
	const dl::Vector3& converted_position,
	const int leg_index, 
	const dl::Vector3& center_of_mass_global,
	const dl::EulerXYZ& robot_rot, 
	const bool consider_rot) const
{
	//leg_index�� 0�`5 �ł���D
	assert(0 <= leg_index);
	assert(leg_index < HexapodConst::kLegNum);

	if (consider_rot)
	{
		return dl::RotateVector3(converted_position - center_of_mass_global, robot_rot * -1) - GetLegBasePosRobotCoodinate(leg_index);
	}
	else
	{
		return converted_position - center_of_mass_global - GetLegBasePosRobotCoodinate(leg_index);
	}
}

dl::Vector3 PhantomXMkII::ConvertLegToGlobalCoordinate(
	const dl::Vector3& converted_position,
	int leg_index,
	const dl::Vector3& center_of_mass_global,
	const dl::EulerXYZ& robot_rot,
	const bool consider_rot) const
{
	//leg_index�� 0�`5 �ł���D
	assert(0 <= leg_index);
	assert(leg_index < HexapodConst::kLegNum);

	if (consider_rot)
	{
		return dl::RotateVector3(converted_position + GetLegBasePosRobotCoodinate(leg_index), robot_rot) + center_of_mass_global;
	}
	else
	{
		return converted_position + GetLegBasePosRobotCoodinate(leg_index) + center_of_mass_global;
	}
}

designlab::Vector3 PhantomXMkII::ConvertRobotToGlobalCoordinate(
	const designlab::Vector3& converted_position,
	const designlab::Vector3& center_of_mass_global,
	const designlab::EulerXYZ& robot_rot,
	const bool consider_rot) const
{
	if (consider_rot)
	{
		return designlab::RotateVector3(converted_position, robot_rot) + center_of_mass_global;
	}
	else
	{
		return converted_position + center_of_mass_global;
	}
}


designlab::Vector3 PhantomXMkII::GetFreeLegPosLegCoodinate(const int leg_index) const noexcept
{
	//leg_index�� 0�`5 �ł���D
	assert(0 <= leg_index);
	assert(leg_index < HexapodConst::kLegNum);

	return free_leg_pos_leg_coordinate_[leg_index];
}

designlab::Vector3 PhantomXMkII::GetLegBasePosRobotCoodinate(const int leg_index) const noexcept
{
	//leg_index�� 0�`5 �ł���D
	assert(0 <= leg_index);
	assert(leg_index < HexapodConst::kLegNum);

	return leg_base_pos_robot_coordinate_[leg_index];
}


bool PhantomXMkII::IsLegInRange(const int leg_index, const designlab::Vector3& leg_pos) const
{
	//leg_index�� 0�`5 �ł���D
	assert(0 <= leg_index);
	assert(leg_index < HexapodConst::kLegNum);

	const dl::Vector2 leg_pos_xy = leg_pos.ProjectedXY();	//���˂����r����W������D

	//�r�̊p�x���͈͓��ɂ��邩���ׂ�D�O�όv�Z�ŊԂɂ��邩���ׂ�
	if (kMinLegPosXY[leg_index].Cross(leg_pos_xy) < 0.0f) { return false; }
	if (kMaxLegPosXY[leg_index].Cross(leg_pos_xy) > 0.0f) { return false; }


	//�r��L�΂����Ƃ̂ł��Ȃ��͈͂ɐL�΂��Ă��Ȃ������ׂ�D
	if (static_cast<int>(leg_pos.z) < -kMaxLegRSize || 0 < static_cast<int>(leg_pos.z)) { return false; }

	if (leg_pos_xy.GetSquaredLength() < dlm::Squared(kMinLegR)) { return false; }

	if (dlm::Squared(kMaxLegR[-static_cast<int>(leg_pos.z)]) < leg_pos_xy.GetSquaredLength()) { return false; }

	return true;
}

bool PhantomXMkII::IsLegInterfering(const std::array<designlab::Vector3, HexapodConst::kLegNum>& leg_pos) const
{
	//�d�S�����_�Ƃ����C���W�n�ɂ����āC�r�̊��𒲂ׂ�D

	//�r�̊��𒲂ׂ�D
	designlab::Vector2 leg_pos_xy[HexapodConst::kLegNum];
	designlab::Vector2 joint_pos_xy[HexapodConst::kLegNum];

	for (int i = 0; i < HexapodConst::kLegNum; i++)
	{
		joint_pos_xy[i] = GetLegBasePosRobotCoodinate(i).ProjectedXY();
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

float PhantomXMkII::GetGroundHeightMarginMin() const noexcept
{
	return kBodyLiftingHeightMin;
}

float PhantomXMkII::GetGroundHeightMarginMax() const noexcept
{
	return kBodyLiftingHeightMax;
}

float PhantomXMkII::CalculateStabilityMargin(const::designlab::leg_func::LegStateBit& leg_state, const std::array<designlab::Vector3, HexapodConst::kLegNum>& leg_pos) const
{
	// std::min ���J�b�R�ň͂�ł���̂́C�}�N���� min �Ɣ�邽�߁D(std::min) �Ə����Ɩ��O���Փ˂��Ȃ�

	std::array<designlab::Vector2, HexapodConst::kLegNum> ground_leg_pos;	// xy���ʂɓ��˂����C�d�S�����_�Ƃ������[�J��(���{�b�g)���W�n�ŁC�r�̈ʒu���v�Z����D
	int ground_leg_pos_num = 0;												// ���x�̊֌W�� vector�łȂ�array���g���D

	//�ڒn�r�̂ݒǉ�����
	for (int i = 0; i < HexapodConst::kLegNum; i++)
	{
		if (dllf::IsGrounded(leg_state, i))
		{
			ground_leg_pos[ground_leg_pos_num] = leg_pos[i].ProjectedXY() + GetLegBasePosRobotCoodinate(i).ProjectedXY();
			ground_leg_pos_num++;
		}
	}


	float min_margin = 0;	// ���p�`�̕ӂƏd�S�̋����̍ŏ��l
	bool is_first = true;	// ���񂩂ǂ����C�ŏ��͕K���l���X�V����

	for (int i = 0; i < ground_leg_pos_num; i++)
	{
		designlab::Vector2 i_to_i_plus_1 = ground_leg_pos[(i + 1) % ground_leg_pos_num] - ground_leg_pos[i];
		i_to_i_plus_1.GetNormalized();
		designlab::Vector2 i_to_com = designlab::Vector2{ 0,0 } - ground_leg_pos[i];

		float margin = i_to_com.Cross(i_to_i_plus_1);	// ���p�`�̕ӂƏd�S�̋���(�ÓI����]�T)

		if (is_first)
		{
			min_margin = margin;
			is_first = false;
		}
		else
		{
			min_margin = (std::min)(min_margin, margin);
		}
	}

	return min_margin;
}

bool PhantomXMkII::IsStable(const::designlab::leg_func::LegStateBit& leg_state, const std::array<designlab::Vector3, HexapodConst::kLegNum>& leg_pos) const
{
	// kStableMargin �ȏ�̗]�T�����邩���ׂ�
	return CalculateStabilityMargin(leg_state, leg_pos) > kStableMargin;
}

bool PhantomXMkII::IsBodyInterferingWithGround(const RobotStateNode& node, const DevideMapState& devide_map) const
{
	float top_z = -10000.0f;	//�n�ʂƂ̌�_�̂����ł��������̂��i�[����

	for (int i = 0; i < HexapodConst::kLegNum; i++)
	{
		//�r�̍����̍��W(�O���[�o��)���擾����
		const designlab::Vector3 kCoxaPos = ConvertRobotToGlobalCoordinate(
			GetLegBasePosRobotCoodinate(i), node.global_center_of_mass, node.rot, false
		);

		if (devide_map.IsInMap(kCoxaPos))
		{
			const float map_top_z = devide_map.GetTopZ(devide_map.GetDevideMapIndexX(kCoxaPos.x), devide_map.GetDevideMapIndexY(kCoxaPos.y));

			top_z = (std::max)(top_z, map_top_z);	//�ł������_�����߂�		
		}
	}

	if (top_z + GetGroundHeightMarginMin() - dlm::kAllowableError < node.global_center_of_mass.z)
	{
		return false;
	}

	return true;
}


std::array<float, PhantomXMkII::kMaxLegRSize> PhantomXMkII::InitMaxLegR() const
{
	// �t�^���wcoxa�Ȃ��̌v�Z���ʂ�p���ď��^���w���v�Z����
	std::array <float, kMaxLegRSize> res;

	for (auto i : res) { i = 0; }

	const float PERMISSION = 0.5f;			//�t�^���w�Ɖ^���w���s�������ʂ����aPermission^0.5�̉~�̒��Ȃ瓙�����ƍl����

	const float mins[3] = { -1.428f, -1.780f, -1.194f };	// �r���͈� �����炭rad �ϊ��������(-81.8�� -101.98�� -68.41��)  190527
	const float maxs[3] = { 1.402f,  1.744f,  1.769f };		// ������coxa,femur,tibia (80.32�� 99.92�� 101.36��)

	//ans of kinematics use sorution of i_kinematics 

	//z�͍ő�196�Dix�͍ő�248
	for (int iz = 0; iz < 200; iz++)
	{
		for (int ix = 53; ix < 248; ix++)
		{
			const designlab::Vector3 line_end((float)ix, 0.0f, (float)iz);		//�r����W�i���[�J���j

			//�t�^���wcoxa�Ȃ�

			//const float _coxa_angle = atan2(line_end.x, line_end.y);	//coxa�p�x

			const float _IK_trueX = sqrt(dlm::Squared(line_end.x) + dlm::Squared(line_end.y)) - PhantomXMkIIConst::kCoxaLength;	//femur���瑫��܂ł����ԃx�N�g����xy���ʂɓ��e�����Ƃ��̃x�N�g���̑傫��
			float _im = sqrt(dlm::Squared(_IK_trueX) + dlm::Squared(line_end.z));					//��΂ɐ�
			if (_im == 0.0f) _im += 0.01f;	//0����΍�

			const float _q1 = -atan2(line_end.z, _IK_trueX);													//�}�C�i�X�ł������W�n�I��q1���̂͏�ɕ�//x���[�����ƒ�`��G���[
			const float _q2 = acos(
				(dlm::Squared(PhantomXMkIIConst::kFemurLength) + dlm::Squared(_im) - dlm::Squared(PhantomXMkIIConst::kTibiaLength)) /
				(2.0f * PhantomXMkIIConst::kFemurLength * _im)
			);	//im=0���ƒ�`��G���[

			const float _femur_angle = _q1 + _q2;
			const float _tibia_angle = acos(
				(dlm::Squared(PhantomXMkIIConst::kFemurLength) + dlm::Squared(PhantomXMkIIConst::kTibiaLength) - dlm::Squared(_im)) /
				(2.0f * PhantomXMkIIConst::kFemurLength * PhantomXMkIIConst::kTibiaLength)
			) - dlm::kFloatPi / 2.0f;

			//float im = sqrt(pow(fabs(IK_trueX), 2.0) + pow(fabs(LineEnd.z), 2.0));//femur���瑫��̋���
			//float d1 = pow((float)L_FEMUR, 2.0) - pow((float)L_TIBIA, 2.0) + pow(fabs((float)im), 2.0);
			//float d2 = 2 * L_FEMUR*im;
			//float q2 = acos((float)d1 / (float)d2);	//�]���藝
			//d1 = pow((float)L_FEMUR, 2.0) - pow(fabs((float)im), 2.0) + pow((float)L_TIBIA, 2.0);
			//d2 = 2 * L_TIBIA*L_FEMUR;
			//tibia = acos((float)d1 / (float)d2) - 1.570796326795;

			//lange of motion
			//���@�͂킩��񂪁A�V�~�����[�V�������ƁA���ꂪ����Ȃ��B
			//if�������ƁA�d�S�Ƒ��捂���̍����A73mm�ȉ��͎��Ȃ��Bhato
			//if ( femur < femurMins)break;
			//if (femurMaxs < femur)break;
			//if (tibia < tibiaMins)break;
			//if(tibiaMaxs < tibia )break;

			//�^���w
			const float _K_trueX = PhantomXMkIIConst::kFemurLength * cos(_femur_angle) + PhantomXMkIIConst::kTibiaLength * cos(_femur_angle + _tibia_angle - dlm::kFloatPi / 2.0f);

			designlab::Vector3 _kinematics;
			_kinematics.x = _K_trueX + PhantomXMkIIConst::kCoxaLength;
			_kinematics.y = 0.0f;
			_kinematics.z = -(PhantomXMkIIConst::kFemurLength * sin(_femur_angle) + PhantomXMkIIConst::kTibiaLength * sin(_femur_angle + _tibia_angle - dlm::kFloatPi / 2.0f));

			const float _Permission = (_kinematics - line_end).GetSquaredLength();

			if (PERMISSION > _Permission)
			{
				constexpr float kLegRom_RMargin = 10.f;
				res[iz] = static_cast<float>(ix) - kLegRom_RMargin;//y=0�̂Ƃ��C�r����z�̂Ƃ���x�����̍ő�͈̔�

#ifdef  MAX_LEG_RADIUS
				if (iz <= 115) { res[iz] = MAX_LEG_RADIUS; }//�r��u���ʒu����������ƃg���N������Ȃ��Ē��ݍ��݂�����������200�܂łɂ���2020/11/09hato
#endif
			}
		}
	}

	return res;
}

std::array<dl::Vector2, HexapodConst::kLegNum> PhantomXMkII::InitMinLegPosXY() const
{
	std::array<dl::Vector2, HexapodConst::kLegNum> res;

	for (int i = 0; i < HexapodConst::kLegNum; i++)
	{
		const float angle = kMovableCoxaAngleMin + PhantomXMkIIConst::kCoxaDefaultAngle[i];
		res[i] = dl::Vector2(cos(angle), sin(angle)).GetNormalized();
	}

	return res;
}

std::array<dl::Vector2, HexapodConst::kLegNum> PhantomXMkII::InitMaxLegPosXY() const
{
	std::array<dl::Vector2, HexapodConst::kLegNum> res;

	for (int i = 0; i < HexapodConst::kLegNum; i++)
	{
		const float angle = kMovableCoxaAngleMax + PhantomXMkIIConst::kCoxaDefaultAngle[i];
		res[i] = dl::Vector2(cos(angle), sin(angle)).GetNormalized();
	}

	return res;
}