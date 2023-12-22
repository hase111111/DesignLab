﻿#include "node_initializer.h"

#include "math_util.h"
#include "leg_state.h"
#include "phantomx_mk2_const.h"


namespace designlab
{

NodeInitializer::NodeInitializer(const::designlab::Vector3& pos, ::designlab::enums::HexapodMove move) : pos_(pos), move_(move)
{
}

RobotStateNode NodeInitializer::InitNode() const
{
	RobotStateNode res;

	//脚状態
	res.leg_state = leg_func::MakeLegStateBit(
		enums::DiscreteComPos::kCenterBack,
		{ true, true, true, true, true, true },
		{ enums::DiscreteLegPos::kCenter, enums::DiscreteLegPos::kCenter, enums::DiscreteLegPos::kCenter,
		enums::DiscreteLegPos::kCenter, enums::DiscreteLegPos::kCenter, enums::DiscreteLegPos::kCenter }
	);


	const float base_z = 0.0f;			// 地面のZ座標	
	const float com_z = pos_.z + base_z;	// ロボットの重心のZ座標

	for (int i = 0; i < HexapodConst::kLegNum; i++)
	{
		res.leg_pos[i] = res.leg_reference_pos[i] = {
			170.f * cos(PhantomXMkIIConst::kCoxaDefaultAngle[i]),
			170.f * sin(PhantomXMkIIConst::kCoxaDefaultAngle[i]),
			-com_z
		};
	}

	res.global_center_of_mass = pos_;

	//ロールピッチヨーで回転を表現する．ロボットの重心を中心にして回転する． 
	res.quat = Quaternion::MakeByAngleAxis(0.f, Vector3::GetUpVec());

	res.next_move = move_;
	res.parent_index = -1;
	res.depth = 0;

	return res;
}

} // namespace designlab