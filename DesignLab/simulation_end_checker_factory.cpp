﻿#include "simulation_end_checker_factory.h"

#include "cassert_define.h"
#include "math_quaternion.h"
#include "math_rot_converter.h"
#include "simulation_end_checker_by_goal_tape.h"
#include "simulation_end_checker_by_position.h"
#include "simulation_end_checker_by_posture.h"

namespace designlab
{

std::unique_ptr<ISimulationEndChecker> SimulationEndCheckerFactory::Create(const SimulationSettingRecord& record)
{
	if (record.end_check_mode == enums::SimulationEndCheckMode::kGoalTape)
	{
		auto simulation_end_checker = std::make_unique<SimulationEndCheckerByGoalTape>(record.goal_tape_position_x);

		return std::move(simulation_end_checker);
	}
	else if (record.end_check_mode == enums::SimulationEndCheckMode::kPosture)
	{
		EulerXYZ target_eular_rad{
			math_util::ConvertDegToRad(record.target_posture.x_angle),
			math_util::ConvertDegToRad(record.target_posture.y_angle),
			math_util::ConvertDegToRad(record.target_posture.z_angle)
		};

		auto simulation_end_checker = std::make_unique<SimulationEndCheckerByPosture>(
			ToQuaternion(target_eular_rad),
			math_util::ConvertDegToRad(record.target_posture_allowable_error_deg)
		);

		return std::move(simulation_end_checker);
	}
	else if (record.end_check_mode == enums::SimulationEndCheckMode::kPosition)
	{
		const Vector3 goal_position(record.target_position);

		auto simulation_end_checker = std::make_unique<SimulationEndCheckerByPosition>(goal_position, record.target_position_allowable_error);

		return std::move(simulation_end_checker);
	}
	else
	{
		assert(false);
	}

	return nullptr;
}

} // namespace designlab