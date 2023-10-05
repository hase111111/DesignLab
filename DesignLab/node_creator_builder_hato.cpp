#include "node_creator_builder_hato.h"

#include "cassert_define.h"
#include "com_move_node_creator_hato.h"
#include "com_up_down_node_creator.h"
#include "leg_hierarchy_node_creator.h"
#include "leg_up_down_node_creator.h"


void NodeCreatorBuilderHato::Build(const DevideMapState& map, const std::shared_ptr<const AbstractHexapodStateCalculator>& calculator_ptr,
	std::map<HexapodMove, std::unique_ptr<INodeCreator>>* node_creator) const
{
	assert(node_creator != nullptr);	// node_creator��nullptr�łȂ��D
	assert(node_creator->size() == 0);	// node_creator�͋�łȂ���΂Ȃ�Ȃ��D
	assert(calculator_ptr);				// calculator_ptr��null�łȂ��D

	(*node_creator)[HexapodMove::kLegHierarchyChange] = std::make_unique<LegHierarchyNodeCreator>(HexapodMove::kLegUpDown);
	(*node_creator)[HexapodMove::kLegUpDown] = std::make_unique<LegUpDownNodeCreator>(map, calculator_ptr, HexapodMove::kComUpDown);
	(*node_creator)[HexapodMove::kComUpDown] = std::make_unique<ComUpDownNodeCreator>(map, calculator_ptr, HexapodMove::kComMove);
	(*node_creator)[HexapodMove::kComMove] = std::make_unique<ComMoveNodeCreatorHato>(map, calculator_ptr, HexapodMove::kLegHierarchyChange);
}
