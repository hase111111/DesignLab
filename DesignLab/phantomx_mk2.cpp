﻿#include "phantomx_mk2.h"

#include <cmath>

#include "cassert_define.h"
#include "designlab_line_segment2.h"
#include "designlab_math_util.h"
#include "leg_state.h"
#include "phantomx_mk2_const.h"


namespace designlab
{

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
		{ PhantomXMkIIConst::kCoxaBaseOffsetX, -PhantomXMkIIConst::kCoxaBaseOffsetY, PhantomXMkIIConst::kCoxaBaseOffsetZ },	// 脚0 右上
		{ 0.0f, -PhantomXMkIIConst::kCenterCoxaBaseOffsetY, PhantomXMkIIConst::kCoxaBaseOffsetZ },						// 脚1 右横
		{ -PhantomXMkIIConst::kCoxaBaseOffsetX, -PhantomXMkIIConst::kCoxaBaseOffsetY, PhantomXMkIIConst::kCoxaBaseOffsetZ },// 脚2 右下
		{ -PhantomXMkIIConst::kCoxaBaseOffsetX, PhantomXMkIIConst::kCoxaBaseOffsetY, PhantomXMkIIConst::kCoxaBaseOffsetZ },	// 脚3 左下
		{ 0.0f, PhantomXMkIIConst::kCenterCoxaBaseOffsetY, PhantomXMkIIConst::kCoxaBaseOffsetZ },						// 脚4 左横
		{ PhantomXMkIIConst::kCoxaBaseOffsetX, PhantomXMkIIConst::kCoxaBaseOffsetY, PhantomXMkIIConst::kCoxaBaseOffsetZ },	// 脚5 左上
	} }),
	kMaxLegRArray(InitMaxLegR()),
	kMinLegPosXY(InitMinLegPosXY()),
	kMaxLegPosXY(InitMaxLegPosXY())
{
}


std::array<HexapodJointState, HexapodConst::kLegNum> PhantomXMkII::CalculateAllJointState(const RobotStateNode& node) const noexcept
{
	std::array<HexapodJointState, HexapodConst::kLegNum> joint_state;

	//計算を行う．
	for (int i = 0; i < HexapodConst::kLegNum; i++)
	{
		joint_state[i] = CalculateJointState(i, node.leg_pos[i]);
	}

	return joint_state;
}

HexapodJointState PhantomXMkII::CalculateJointState(const int leg_index, const Vector3& leg_pos) const noexcept
{
	//leg_indexは 0～5 である．
	assert(0 <= leg_index);
	assert(leg_index < HexapodConst::kLegNum);

	//各パラメータを初期化する
	HexapodJointState res;

	const int kJointNum = 4;
	const int kJointAngleNum = kJointNum - 1;

	res.joint_pos_leg_coordinate.clear();
	res.joint_angle.clear();

	res.joint_pos_leg_coordinate.resize(kJointNum);
	res.joint_angle.resize(kJointAngleNum);


	// coxa jointの計算
	res.joint_pos_leg_coordinate[0] = Vector3{ 0, 0, 0 };	//脚座標系ではcoxa jointは原点にある．

	// 脚先の追加
	res.joint_pos_leg_coordinate[3] = leg_pos;

	// coxa angleの計算
	{
		float coxa_joint_angle = std::atan2f(leg_pos.y, leg_pos.x);

		if (leg_pos.y == 0 && leg_pos.x == 0) { coxa_joint_angle = PhantomXMkIIConst::kCoxaDefaultAngle[leg_index]; }

		if (!PhantomXMkIIConst::IsVaildCoxaAngle(leg_index, coxa_joint_angle))
		{
			//範囲外ならば，180度回転させた時に範囲内にあるかを調べる．
			if (PhantomXMkIIConst::IsVaildCoxaAngle(leg_index, coxa_joint_angle + math_util::kFloatPi))
			{
				coxa_joint_angle += math_util::kFloatPi;
			}
			else if (PhantomXMkIIConst::IsVaildCoxaAngle(leg_index, coxa_joint_angle - math_util::kFloatPi))
			{
				coxa_joint_angle -= math_util::kFloatPi;
			}
		}

		res.joint_angle[0] = coxa_joint_angle;
	}

	// femur jointの計算
	{
		const Vector3 femur_joint_pos = Vector3{
			PhantomXMkIIConst::kCoxaLength * std::cos(res.joint_angle[0]),
			PhantomXMkIIConst::kCoxaLength * std::sin(res.joint_angle[0]),
			0
		};

		res.joint_pos_leg_coordinate[1] = femur_joint_pos;

		if (!math_util::CanMakeTriangle((leg_pos - femur_joint_pos).GetLength(), PhantomXMkIIConst::kFemurLength, PhantomXMkIIConst::kTibiaLength))
		{
			// そもそも脚先が脚の付け根から届かない場合，一番近い位置まで脚を伸ばす．

			const float angle_ft = std::atan2(leg_pos.z - femur_joint_pos.z, (leg_pos.ProjectedXY() - femur_joint_pos.ProjectedXY()).GetLength());

			// angle_ftの位相を180度回転する．-180度～180度の範囲にする．
			float angle_ft_phase = angle_ft + math_util::kFloatPi;
			angle_ft_phase = angle_ft_phase > math_util::kFloatPi ? angle_ft_phase - math_util::kFloatPi : angle_ft_phase;

			const Vector3 candidate_leg_pos = femur_joint_pos + Vector3{
				(PhantomXMkIIConst::kFemurLength + PhantomXMkIIConst::kTibiaLength)* std::cos(res.joint_angle[0])* std::cos(angle_ft),
					(PhantomXMkIIConst::kFemurLength + PhantomXMkIIConst::kTibiaLength)* std::sin(res.joint_angle[0])* std::cos(angle_ft),
					(PhantomXMkIIConst::kFemurLength + PhantomXMkIIConst::kTibiaLength)* std::sin(angle_ft)
			};

			const Vector3 candidate_leg_pos_phase = femur_joint_pos + Vector3{
				(PhantomXMkIIConst::kFemurLength* std::cos(angle_ft_phase) + PhantomXMkIIConst::kTibiaLength * std::cos(angle_ft))* std::cos(res.joint_angle[0]),
					(PhantomXMkIIConst::kFemurLength* std::cos(angle_ft_phase) + PhantomXMkIIConst::kTibiaLength * std::cos(angle_ft))* std::sin(res.joint_angle[0]),
					PhantomXMkIIConst::kFemurLength* std::sin(angle_ft_phase) + PhantomXMkIIConst::kTibiaLength * std::sin(angle_ft)
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
				angle_t = -math_util::kFloatPi;
			}

			res.joint_angle[1] = angle_f;
			res.joint_angle[2] = angle_t;

			res.joint_pos_leg_coordinate[2] = femur_joint_pos + Vector3{
				PhantomXMkIIConst::kFemurLength* std::cos(angle_f)* std::cos(res.joint_angle[0]),
					PhantomXMkIIConst::kFemurLength* std::cos(angle_f)* std::sin(res.joint_angle[0]),
					PhantomXMkIIConst::kFemurLength* std::sin(angle_f)
			};

			res.joint_pos_leg_coordinate[3] = res.joint_pos_leg_coordinate[2] + Vector3{
				PhantomXMkIIConst::kTibiaLength* std::cos(angle_f + angle_t)* std::cos(res.joint_angle[0]),
					PhantomXMkIIConst::kTibiaLength* std::cos(angle_f + angle_t)* std::sin(res.joint_angle[0]),
					PhantomXMkIIConst::kTibiaLength* std::sin(angle_f + angle_t)
			};

			res.is_in_range = false;	//範囲外であることを示す．

			return res;
		}
	}


	// femur angle の計算
	const Vector3 femur_to_leg = leg_pos - res.joint_pos_leg_coordinate[1];		//脚先から第一関節までの長さ．
	const float femur_to_leg_x = femur_to_leg.ProjectedXY().GetLength() * 																//脚先へ向かう方向をxの正方向にする座標系に置き換える 			
		((leg_pos.ProjectedXY().GetSquaredLength() > res.joint_pos_leg_coordinate[1].ProjectedXY().GetSquaredLength()) ? 1.f : -1.f);	//脚先が第一関節よりも近い場合は正の方向にする．
	const float femur_to_leg_z = femur_to_leg.z;

	{
		const float arccos_upper = femur_to_leg.GetSquaredLength() + math_util::Squared(PhantomXMkIIConst::kFemurLength) - math_util::Squared(PhantomXMkIIConst::kTibiaLength);
		const float arccos_lower = 2 * PhantomXMkIIConst::kFemurLength * femur_to_leg.GetLength();
		const float arccos_arg = arccos_upper / arccos_lower;

		const float fumur_joint_angle = std::acos(arccos_arg) + std::atan2(femur_to_leg_z, femur_to_leg_x);

		res.joint_angle[1] = fumur_joint_angle;
	}

	// tibia jointの計算
	{
		const Vector3 femur_to_tibia = Vector3{
			PhantomXMkIIConst::kFemurLength * std::cos(res.joint_angle[0]) * std::cos(res.joint_angle[1]),
			PhantomXMkIIConst::kFemurLength * std::sin(res.joint_angle[0]) * std::cos(res.joint_angle[1]),
			PhantomXMkIIConst::kFemurLength * std::sin(res.joint_angle[1])
		};

		const Vector3 tibia_joint_pos = res.joint_pos_leg_coordinate[1] + femur_to_tibia;

		res.joint_pos_leg_coordinate[2] = tibia_joint_pos;
	}


	// tibia angleの計算
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

bool PhantomXMkII::IsVaildJointState(const int leg_index, const Vector3& leg_pos, const HexapodJointState& joint_state) const noexcept
{
	assert(joint_state.joint_pos_leg_coordinate.size() == 4);
	assert(joint_state.joint_angle.size() == 3);

	// coxa関節の範囲内に存在しているかを確認する
	if (!PhantomXMkIIConst::IsVaildCoxaAngle(leg_index, joint_state.joint_angle[0])) { return false; }

	// femur関節の範囲内に存在しているかを確認する
	if (!PhantomXMkIIConst::IsVaildFemurAngle(joint_state.joint_angle[1])) { return false; }

	// tibia関節の範囲内に存在しているかを確認する
	if (!PhantomXMkIIConst::IsVaildTibiaAngle(joint_state.joint_angle[2])) { return false; }

	// リンクの長さを確認する
	if (!math_util::IsEqual((joint_state.joint_pos_leg_coordinate[0] - joint_state.joint_pos_leg_coordinate[1]).GetLength(), PhantomXMkIIConst::kCoxaLength))
	{
		return false;
	}

	if (!math_util::IsEqual((joint_state.joint_pos_leg_coordinate[1] - joint_state.joint_pos_leg_coordinate[2]).GetLength(), PhantomXMkIIConst::kFemurLength))
	{
		return false;
	}

	if (!math_util::IsEqual((joint_state.joint_pos_leg_coordinate[2] - joint_state.joint_pos_leg_coordinate[3]).GetLength(), PhantomXMkIIConst::kTibiaLength))
	{
		return false;
	}

	// 脚位置と脚先座標が一致しているかを確認する
	if (joint_state.joint_pos_leg_coordinate[3] != leg_pos) { return false; }

	return true;
}


Vector3 PhantomXMkII::ConvertGlobalToLegCoordinate(
	const Vector3& converted_position,
	const int leg_index,
	const Vector3& center_of_mass_global,
	const Quaternion& robot_quat,
	const bool consider_rot) const
{
	//leg_indexは 0～5 である．
	assert(0 <= leg_index);
	assert(leg_index < HexapodConst::kLegNum);

	if (consider_rot)
	{
		return RotateVector3(converted_position - center_of_mass_global, robot_quat.GetConjugate()) - GetLegBasePosRobotCoodinate(leg_index);
	}
	else
	{
		return converted_position - center_of_mass_global - GetLegBasePosRobotCoodinate(leg_index);
	}
}

Vector3 PhantomXMkII::ConvertLegToGlobalCoordinate(
	const Vector3& converted_position,
	int leg_index,
	const Vector3& center_of_mass_global,
	const Quaternion& robot_quat,
	const bool consider_rot) const
{
	//leg_indexは 0～5 である．
	assert(0 <= leg_index);
	assert(leg_index < HexapodConst::kLegNum);

	if (consider_rot)
	{
		return RotateVector3(converted_position + GetLegBasePosRobotCoodinate(leg_index), robot_quat) + center_of_mass_global;
	}
	else
	{
		return converted_position + GetLegBasePosRobotCoodinate(leg_index) + center_of_mass_global;
	}
}

Vector3 PhantomXMkII::ConvertRobotToGlobalCoordinate(
	const Vector3& converted_position,
	const Vector3& center_of_mass_global,
	const Quaternion& robot_quat,
	const bool consider_rot) const
{
	if (consider_rot)
	{
		return RotateVector3(converted_position, robot_quat) + center_of_mass_global;
	}
	else
	{
		return converted_position + center_of_mass_global;
	}
}

Vector3 PhantomXMkII::ConvertRobotToLegCoordinate(const Vector3& converted_position, const int leg_index) const
{
	return converted_position - GetLegBasePosRobotCoodinate(leg_index);
}

Vector3 PhantomXMkII::ConvertLegToRobotCoordinate(const Vector3& converted_position, const int leg_index) const
{
	return converted_position + GetLegBasePosRobotCoodinate(leg_index);
}


Vector3 PhantomXMkII::GetFreeLegPosLegCoodinate(const int leg_index) const noexcept
{
	//leg_indexは 0～5 である．
	assert(0 <= leg_index);
	assert(leg_index < HexapodConst::kLegNum);

	return free_leg_pos_leg_coordinate_[leg_index];
}

Vector3 PhantomXMkII::GetLegBasePosRobotCoodinate(const int leg_index) const noexcept
{
	//leg_indexは 0～5 である．
	assert(0 <= leg_index);
	assert(leg_index < HexapodConst::kLegNum);

	return leg_base_pos_robot_coordinate_[leg_index];
}


bool PhantomXMkII::IsLegInRange(const int leg_index, const Vector3& leg_pos) const
{
	//leg_indexは 0～5 である．
	assert(0 <= leg_index);
	assert(leg_index < HexapodConst::kLegNum);

	if (GetFreeLegPosLegCoodinate(leg_index).z < leg_pos.z) { return false; }

	const Vector2 leg_pos_xy = leg_pos.ProjectedXY();	//投射した脚先座標をえる．

	//脚の角度が範囲内にあるか調べる．外積計算で間にあるか調べる
	if (kMinLegPosXY[leg_index].Cross(leg_pos_xy) < 0.0f) { return false; }
	if (kMaxLegPosXY[leg_index].Cross(leg_pos_xy) > 0.0f) { return false; }


	//脚を伸ばすことのできない範囲に伸ばしていないか調べる．
	if (static_cast<int>(leg_pos.z) < -kMaxLegRSize || 0 < static_cast<int>(leg_pos.z)) { return false; }

	if (leg_pos_xy.GetSquaredLength() < math_util::Squared(kMinLegR)) { return false; }

	if (math_util::Squared(kMaxLegR) < leg_pos_xy.GetSquaredLength()) { return false; }

	if (math_util::Squared(kMaxLegRArray[-static_cast<int>(leg_pos.z)]) < leg_pos_xy.GetSquaredLength()) { return false; }

	return true;
}

bool PhantomXMkII::IsAllLegInRange(const leg_func::LegStateBit leg_state, const std::array<Vector3, HexapodConst::kLegNum>& leg_pos) const
{
	for (int i = 0; i < HexapodConst::kLegNum; i++)
	{
		if (leg_func::IsGrounded(leg_state, i) && !IsLegInRange(i, leg_pos[i]))
		{
			return false;
		}
	}

	return true;
}

bool PhantomXMkII::IsLegInterfering(const std::array<Vector3, HexapodConst::kLegNum>& leg_pos) const
{
	//重心を原点とした，座標系において，脚の干渉を調べる．

	//脚の干渉を調べる．
	Vector2 leg_pos_xy[HexapodConst::kLegNum];
	Vector2 joint_pos_xy[HexapodConst::kLegNum];

	for (int i = 0; i < HexapodConst::kLegNum; i++)
	{
		joint_pos_xy[i] = GetLegBasePosRobotCoodinate(i).ProjectedXY();
		leg_pos_xy[i] = leg_pos[i].ProjectedXY() + joint_pos_xy[i];
	}

	//隣の脚との干渉を調べる．
	for (int i = 0; i < HexapodConst::kLegNum; i++)
	{
		LineSegment2 line1(joint_pos_xy[i], leg_pos_xy[i]);
		LineSegment2 line2(joint_pos_xy[(i + 1) % HexapodConst::kLegNum], leg_pos_xy[(i + 1) % HexapodConst::kLegNum]);

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

float PhantomXMkII::CalculateStabilityMargin(const leg_func::LegStateBit& leg_state, const std::array<Vector3, HexapodConst::kLegNum>& leg_pos) const
{
	// std::min をカッコで囲んでいるのは，マクロの min と被るため．(std::min) と書くと名前が衝突しない

	std::array<Vector2, HexapodConst::kLegNum> ground_leg_pos;	// xy平面に投射した，重心を原点としたローカル(ロボット)座標系で，脚の位置を計算する．
	int ground_leg_pos_num = 0;												// 速度の関係上 vectorでなくarrayを使う．

	//接地脚のみ追加する
	for (int i = 0; i < HexapodConst::kLegNum; i++)
	{
		if (leg_func::IsGrounded(leg_state, i))
		{
			ground_leg_pos[ground_leg_pos_num] = leg_pos[i].ProjectedXY() + GetLegBasePosRobotCoodinate(i).ProjectedXY();
			ground_leg_pos_num++;
		}
	}


	float min_margin = 0;	// 多角形の辺と重心の距離の最小値
	bool is_first = true;	// 初回かどうか，最初は必ず値を更新する

	for (int i = 0; i < ground_leg_pos_num; i++)
	{
		Vector2 i_to_i_plus_1 = ground_leg_pos[(i + 1) % ground_leg_pos_num] - ground_leg_pos[i];
		i_to_i_plus_1.GetNormalized();
		Vector2 i_to_com = Vector2{ 0,0 } - ground_leg_pos[i];

		float margin = i_to_com.Cross(i_to_i_plus_1);	// 多角形の辺と重心の距離(静的安定余裕)

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

bool PhantomXMkII::IsStable(const leg_func::LegStateBit& leg_state, const std::array<Vector3, HexapodConst::kLegNum>& leg_pos) const
{
	// kStableMargin 以上の余裕があるか調べる
	return CalculateStabilityMargin(leg_state, leg_pos) > kStableMargin;
}

bool PhantomXMkII::IsBodyInterferingWithGround(const RobotStateNode& node, const DevideMapState& devide_map) const
{
	//重心の干渉を調べる．
	{
		const float top_z = (std::max)(
			devide_map.GetMapMinZ(),
			devide_map.GetTopZ(devide_map.GetDevideMapIndexX(node.global_center_of_mass.x), devide_map.GetDevideMapIndexY(node.global_center_of_mass.y))
			);

		if (top_z != devide_map.GetMapMinZ() && top_z + GetGroundHeightMarginMin() > node.global_center_of_mass.z)
		{
			return true;
		}
	}

	for (int i = 0; i < HexapodConst::kLegNum; i++)
	{
		//脚の根元の座標(グローバル)を取得する．
		{
			const Vector3 coxa_pos_global_coord = ConvertRobotToGlobalCoordinate(
				GetLegBasePosRobotCoodinate(i), node.global_center_of_mass, node.quat, true
			);

			if (devide_map.IsInMap(coxa_pos_global_coord))
			{
				const float coxa_top_z = (std::max)(
					devide_map.GetTopZ(devide_map.GetDevideMapIndexX(coxa_pos_global_coord.x), devide_map.GetDevideMapIndexY(coxa_pos_global_coord.y)),
					devide_map.GetMapMinZ()
					);

				if (coxa_top_z != devide_map.GetMapMinZ() && coxa_top_z + GetGroundHeightMarginMin() > coxa_pos_global_coord.z)
				{
					return true;
				}
			}
		}

		//脚先の座標(グローバル)を取得する．
		Vector3 leg_pos_global_coord = ConvertLegToGlobalCoordinate(
			GetFreeLegPosLegCoodinate(i), i, node.global_center_of_mass, node.quat, true
		);

		if (devide_map.IsInMap(leg_pos_global_coord))
		{
			const float leg_top_z = (std::max)(
				devide_map.GetTopZ(devide_map.GetDevideMapIndexX(leg_pos_global_coord.x), devide_map.GetDevideMapIndexY(leg_pos_global_coord.y)),
				devide_map.GetMapMinZ()
				);

			if (leg_top_z != devide_map.GetMapMinZ() && leg_top_z + GetGroundHeightMarginMin() + GetFreeLegPosLegCoodinate(i).z > leg_pos_global_coord.z)
			{
				return true;
			}
		}
	}

	return false;
}


std::array<float, PhantomXMkII::kMaxLegRSize> PhantomXMkII::InitMaxLegR() const
{
	// 逆運動学coxaなしの計算結果を用いて準運動学を計算する
	std::array <float, kMaxLegRSize> res;

	for (auto i : res) { i = 0; }

	const float PERMISSION = 0.5f;			//逆運動学と運動学を行った結果が半径Permission^0.5の円の中なら等しいと考える

	const float mins[3] = { -1.428f, -1.780f, -1.194f };	// 脚可動範囲 おそらくrad 変換したやつ(-81.8° -101.98° -68.41°)  190527
	const float maxs[3] = { 1.402f,  1.744f,  1.769f };		// 左からcoxa,femur,tibia (80.32° 99.92° 101.36°)

	//ans of kinematics use sorution of i_kinematics 

	//zは最大196．ixは最大248
	for (int iz = 0; iz < 200; iz++)
	{
		for (int ix = 53; ix < 248; ix++)
		{
			const Vector3 line_end((float)ix, 0.0f, (float)iz);		//脚先座標（ローカル）

			//逆運動学coxaなし

			//const float _coxa_angle = atan2(line_end.x, line_end.y);	//coxa角度

			const float _IK_trueX = sqrt(math_util::Squared(line_end.x) + math_util::Squared(line_end.y)) - PhantomXMkIIConst::kCoxaLength;	//femurから足先までを結ぶベクトルをxy平面に投影したときのベクトルの大きさ
			float _im = sqrt(math_util::Squared(_IK_trueX) + math_util::Squared(line_end.z));					//絶対に正
			if (_im == 0.0f) _im += 0.01f;	//0割り対策

			const float _q1 = -atan2(line_end.z, _IK_trueX);													//マイナスでおｋ座標系的にq1自体は常に負//xがゼロだと定義域エラー
			const float _q2 = acos(
				(math_util::Squared(PhantomXMkIIConst::kFemurLength) + math_util::Squared(_im) - math_util::Squared(PhantomXMkIIConst::kTibiaLength)) /
				(2.0f * PhantomXMkIIConst::kFemurLength * _im)
			);	//im=0だと定義域エラー

			const float _femur_angle = _q1 + _q2;
			const float _tibia_angle = acos(
				(math_util::Squared(PhantomXMkIIConst::kFemurLength) + math_util::Squared(PhantomXMkIIConst::kTibiaLength) - math_util::Squared(_im)) /
				(2.0f * PhantomXMkIIConst::kFemurLength * PhantomXMkIIConst::kTibiaLength)
			) - math_util::kFloatPi / 2.0f;

			//float im = sqrt(pow(fabs(IK_trueX), 2.0) + pow(fabs(LineEnd.z), 2.0));//femurから足先の距離
			//float d1 = pow((float)L_FEMUR, 2.0) - pow((float)L_TIBIA, 2.0) + pow(fabs((float)im), 2.0);
			//float d2 = 2 * L_FEMUR*im;
			//float q2 = acos((float)d1 / (float)d2);	//余弦定理
			//d1 = pow((float)L_FEMUR, 2.0) - pow(fabs((float)im), 2.0) + pow((float)L_TIBIA, 2.0);
			//d2 = 2 * L_TIBIA*L_FEMUR;
			//tibia = acos((float)d1 / (float)d2) - 1.570796326795;

			//lange of motion
			//実機はわからんが、シミュレーションだと、これがいらない。
			//if文入れると、重心と足先高さの差が、73mm以下は取れない。hato
			//if ( femur < femurMins)break;
			//if (femurMaxs < femur)break;
			//if (tibia < tibiaMins)break;
			//if(tibiaMaxs < tibia )break;

			//運動学
			const float _K_trueX = PhantomXMkIIConst::kFemurLength * cos(_femur_angle) + PhantomXMkIIConst::kTibiaLength * cos(_femur_angle + _tibia_angle - math_util::kFloatPi / 2.0f);

			Vector3 _kinematics;
			_kinematics.x = _K_trueX + PhantomXMkIIConst::kCoxaLength;
			_kinematics.y = 0.0f;
			_kinematics.z = -(PhantomXMkIIConst::kFemurLength * sin(_femur_angle) + PhantomXMkIIConst::kTibiaLength * sin(_femur_angle + _tibia_angle - math_util::kFloatPi / 2.0f));

			const float _Permission = (_kinematics - line_end).GetSquaredLength();

			if (PERMISSION > _Permission)
			{
				constexpr float kLegRom_RMargin = 10.f;
				res[iz] = static_cast<float>(ix) - kLegRom_RMargin;//y=0のとき，脚高さzのときのx方向の最大の範囲

#ifdef  MAX_LEG_RADIUS
				if (iz <= 115) { res[iz] = MAX_LEG_RADIUS; }//脚を置く位置が遠すぎるとトルクが足りなくて沈み込みが激しいから200までにした2020/11/09hato
#endif
			}
		}
	}

	return res;
}

std::array<Vector2, HexapodConst::kLegNum> PhantomXMkII::InitMinLegPosXY() const
{
	std::array<Vector2, HexapodConst::kLegNum> res;

	for (int i = 0; i < HexapodConst::kLegNum; i++)
	{
		const float angle = kMovableCoxaAngleMin + PhantomXMkIIConst::kCoxaDefaultAngle[i];
		res[i] = Vector2(cos(angle), sin(angle)).GetNormalized();
	}

	return res;
}

std::array<Vector2, HexapodConst::kLegNum> PhantomXMkII::InitMaxLegPosXY() const
{
	std::array<Vector2, HexapodConst::kLegNum> res;

	for (int i = 0; i < HexapodConst::kLegNum; i++)
	{
		const float angle = kMovableCoxaAngleMax + PhantomXMkIIConst::kCoxaDefaultAngle[i];
		res[i] = Vector2(cos(angle), sin(angle)).GetNormalized();
	}

	return res;
}

}	//namespace designlab