﻿//! @file node_creator_body_rot.h
//! @brief ロボットの回転を表すノードを生成するクラス．

#ifndef DESIGNLAB_BODY_YAW_ROT_NODE_CREATOR_H_
#define DESIGNLAB_BODY_YAW_ROT_NODE_CREATOR_H_

#include "interface_node_creator.h"

#include <memory>

#include "designlab_math_util.h"
#include "devide_map_state.h"
#include "interface_hexapod_coordinate_converter.h"
#include "interface_hexapod_vaild_checker.h"


namespace designlab
{

//! @class NodeCreatorBodyRot
//! @brief ロボットの回転を表すノードを生成するクラス．
class NodeCreatorBodyRot final : public INodeCreator
{
public:

	NodeCreatorBodyRot(
		const DevideMapState& devide_map,
		const std::shared_ptr<const IHexapodCoordinateConverter>& converter_ptr,
		const std::shared_ptr<const IHexapodVaildChecker>& checker_ptr,
		const Vector3& rot_axis,
		enums::HexapodMove next_move
	);

	~NodeCreatorBodyRot() = default;

	void Create(const RobotStateNode& current_node, int current_num, std::vector<RobotStateNode>* output_graph) const override;


private:

	static constexpr float kBodyYawRotAngleMax = math_util::ConvertDegToRad(10.f);		//!< ロボットのヨー軸周りの回転角度．
	static constexpr float kBodyYawRotAngleMin = math_util::ConvertDegToRad(-10.f);	//!< ロボットのヨー軸周りの回転角度．
	static constexpr int kBodyYawRotAngleDivNum = 21;	//!< ロボットのヨー軸周りの回転角度の分割数（奇数にすること）．

	constexpr std::array<float, kBodyYawRotAngleDivNum> CreateCandiateAngle() const
	{
		std::array<float, kBodyYawRotAngleDivNum> res{};
		int count = 0;

		res[count++] = (kBodyYawRotAngleMax + kBodyYawRotAngleMin) / 2;

		//絶対値の小さい順に並べる
		const float dif = (kBodyYawRotAngleMax - kBodyYawRotAngleMin) / (kBodyYawRotAngleDivNum - 1);

		for (int i = 0; i < kBodyYawRotAngleDivNum / 2; ++i)
		{
			res[count++] = res[0] + dif * (i + 1);
			res[count++] = res[0] - dif * (i + 1);
		}

		return res;
	}

	const std::array<float, kBodyYawRotAngleDivNum> candiate_angle_ = CreateCandiateAngle();

	const DevideMapState map_;		//!< 地面の状態を格納したクラス．
	const enums::HexapodMove next_move_;	//!< 次の動作．

	const std::shared_ptr<const IHexapodCoordinateConverter> converter_ptr_;
	const std::shared_ptr<const IHexapodVaildChecker> checker_ptr_;

	Vector3 rot_axis_;	//!< 回転軸．

	static_assert(kBodyYawRotAngleMax > kBodyYawRotAngleMin, "kBodyYawRotAngleMax は kBodyYawRotAngleMinよりも大きい必要があります.");
	static_assert(kBodyYawRotAngleDivNum % 2 == 1, "kBodyYawRotAngleDivNum は奇数である必要があります.");
};

}	// namespace designlab


#endif	// DESIGNLAB_BODY_YAW_ROT_NODE_CREATOR_H_