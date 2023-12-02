//! @file gait_pattern_generator_switch_move.h
//! @brief ������ނ̕��e�p�^�[���̐������@��؂�ւ��邽�߂̃N���X�D


#ifndef GAIT_PATTERN_GENERATOR_SWITCH_MOVE_H
#define GAIT_PATTERN_GENERATOR_SWITCH_MOVE_H

#include "interface_gait_pattern_generator.h"

#include <memory>
#include <vector>

#include "graph_tree_creator.h"
#include "interface_gait_pattern_generator.h"
#include "robot_state_node.h"


//! @class GaitPatternGeneratorSwitchMove
//! @brief �ĕ]����@�����������N���X�D
class GaitPatternGeneratorSwitchMove final : public IGaitPatternGenerator
{
public:

	//! @param[in] gait_pattern_generator_ptr �ʏ펞�Ɏg�p������́D
	//! @param[in] gait_pattern_generator_revaluation_ptr �ĕ]�����Ɏg�p������́D
	GaitPatternGeneratorSwitchMove(
		std::unique_ptr<IGaitPatternGenerator>&& gait_pattern_generator_for_straigt,
		std::unique_ptr<IGaitPatternGenerator>&& gait_pattern_generator_for_turn_spot
	);

	~GaitPatternGeneratorSwitchMove() = default;


	GraphSearchResult GetNextNodebyGraphSearch(
		const RobotStateNode& current_node,
		const MapState& map,
		const TargetRobotState& target,
		RobotStateNode* output_node
	) override;

private:

	const std::unique_ptr<IGaitPatternGenerator> gait_pattern_generator_for_straigt_ptr_;
	const std::unique_ptr<IGaitPatternGenerator> gait_pattern_generator_for_turn_spot_ptr_;
};

#endif // GAIT_PATTERN_GENERATOR_SWITCH_MOVE_H