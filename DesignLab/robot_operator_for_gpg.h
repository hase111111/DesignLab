﻿#ifndef DESIGNLAB_ROBOT_OPERATOR_FOR_GPG_H_
#define DESIGNLAB_ROBOT_OPERATOR_FOR_GPG_H_

#include <vector>

#include "designlab_math_util.h"
#include "interface_robot_operator.h"


//! @class RobotOperatorForGpg
//! @brief global path generator を行うための仮置きクラス．
class RobotOperatorForGpg final : public IRobotOperator
{
public:

	RobotOperatorForGpg();

	RobotOperation Init() const override;

	RobotOperation Update(const RobotStateNode& node) override;

private:

	static constexpr float kAllowableAngleError{ ::designlab::math_util::ConvertDegToRad(2.0f) };	//!< 目標角度と現在の角度の許容誤差．

	std::vector<::designlab::Vector3> global_route_;	//!< グローバルパス．
};

#endif  // DESIGNLAB_ROBOT_OPERATOR_FOR_GPG_H_