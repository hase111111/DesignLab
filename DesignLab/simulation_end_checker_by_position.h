﻿//! @file simulation_end_checker_by_position.h
//! @brief 最終位置によるシミュレーション終了判定クラス．

#ifndef	DESIGNLAB_SIMULATION_END_CHECKER_BY_POSITION_H_
#define	DESIGNLAB_SIMULATION_END_CHECKER_BY_POSITION_H_

#include "interface_simulation_end_checker.h"


namespace designlab
{

//! @class SimulationEndCheckerByPosition
//! @brief 最終位置によるシミュレーション終了判定クラス．
class SimulationEndCheckerByPosition final : public ISimulationEndChecker
{
public:

	SimulationEndCheckerByPosition(const ::designlab::Vector3& goal_position, float allowable_error);

	bool IsEnd(const RobotStateNode& node) const override;

private:
	const ::designlab::Vector3 goal_position_;	//!< 目標位置．
	const float allowable_error_;				//!< 許容誤差．
};

}	// namespace designlab


#endif	// DESIGNLAB_SIMULATION_END_CHECKER_BY_POSITION_H_