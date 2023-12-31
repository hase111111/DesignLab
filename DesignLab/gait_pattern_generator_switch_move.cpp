﻿
//! @author    Hasegawa
//! @copyright © 埼玉大学 設計工学研究室 2023. All right reserved.

#include "gait_pattern_generator_switch_move.h"

#include <string>
#include <utility>

#include <magic_enum.hpp>

#include "cassert_define.h"


namespace designlab
{

GaitPatternGeneratorSwitchMove::GaitPatternGeneratorSwitchMove(
  std::unique_ptr<IGaitPatternGenerator>&& gait_pattern_generator_for_straight_ptr,
  std::unique_ptr<IGaitPatternGenerator>&& gait_pattern_generator_for_turn_spot_ptr) :
    gait_pattern_generator_for_straight_ptr_(std::move(gait_pattern_generator_for_straight_ptr)),
    gait_pattern_generator_for_turn_spot_ptr_(std::move(gait_pattern_generator_for_turn_spot_ptr))
{
}

GraphSearchResult GaitPatternGeneratorSwitchMove::GetNextNodeByGraphSearch(
  const RobotStateNode& current_node,
  const MapState& map,
  const RobotOperation& operation,
  RobotStateNode* output_node
)
{
    if (operation.operation_type == enums::RobotOperationType::kStraightMoveVector ||
        operation.operation_type == enums::RobotOperationType::kStraightMovePosition)
    {
        return gait_pattern_generator_for_straight_ptr_->GetNextNodeByGraphSearch(
          current_node,
          map,
          operation,
          output_node);
    }
    else if (operation.operation_type == enums::RobotOperationType::kSpotTurnRotAxis ||
             operation.operation_type == enums::RobotOperationType::kSpotTurnLastPosture)
    {
        return gait_pattern_generator_for_turn_spot_ptr_->GetNextNodeByGraphSearch(
          current_node,
          map,
          operation,
          output_node);
    }
    else
    {
        assert(false);

        std::string error_message = "存在していない動作種別です。";

        return { enums::Result::kFailure, error_message };
    }
}

}  // namespace designlab
