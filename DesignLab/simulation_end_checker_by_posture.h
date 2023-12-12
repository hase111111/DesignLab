﻿//! @file simulation_end_checker_by_posture.h
//! @brief 最終姿勢によって，シミュレーションの終了を判定するクラス．

#ifndef	DESIGNLAB_SIMULATION_END_CHECKER_BY_POSTURE_H_
#define	DESIGNLAB_SIMULATION_END_CHECKER_BY_POSTURE_H_

#include "interface_simulation_end_checker.h"

#include "designlab_euler.h"
#include "designlab_quaternion.h"


//! @class SimulationEndCheckerByPosture
//! @brief 最終姿勢によるシミュレーション終了判定クラス．
class SimulationEndCheckerByPosture final : public ISimulationEndChecker
{
public:

	SimulationEndCheckerByPosture(const ::designlab::Quaternion& goal_orientation, float allowable_error);

	bool IsEnd(const RobotStateNode& node) const override;

private:

	const ::designlab::Quaternion goal_orientation_;	//!< 目標姿勢
	const ::designlab::EulerXYZ goal_euler_;			//!< 目標姿勢

	const float allowable_error_;	//!< 許容誤差
};

#endif	// DESIGNLAB_SIMULATION_END_CHECKER_BY_POSTURE_H_