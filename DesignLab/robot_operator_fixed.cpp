﻿#include "robot_operator_fixed.h"


namespace designlab
{

RobotOperatorFixed::RobotOperatorFixed(const RobotOperation& operation) : operation_(operation)
{
}

RobotOperation RobotOperatorFixed::Init() const
{
	return operation_;
}

RobotOperation RobotOperatorFixed::Update([[maybe_unused]] const RobotStateNode& state)
{
	return operation_;
}

} // namespace designlab