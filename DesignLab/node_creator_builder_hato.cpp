﻿#include "node_creator_builder_hato.h"

#include "cassert_define.h"
#include "com_move_node_creator_hato.h"
#include "com_up_down_node_creator.h"
#include "leg_hierarchy_node_creator.h"
#include "leg_up_down_node_creator.h"


NodeCreatorBuilderHato::NodeCreatorBuilderHato(
	const std::shared_ptr<const IHexapodCoordinateConverter>& converter_ptr, 
	const std::shared_ptr<const IHexapodStatePresenter>& presenter_ptr, 
	const std::shared_ptr<const IHexapodVaildChecker>& checker_ptr) : 
	converter_ptr_(converter_ptr),
	presenter_ptr_(presenter_ptr),
	checker_ptr_(checker_ptr)
{
}

void NodeCreatorBuilderHato::Build(
	const DevideMapState& map,
	std::map<HexapodMove, std::unique_ptr<INodeCreator>>* node_creator) const
{
	assert(node_creator != nullptr);	// node_creatorがnullptrでない．
	assert(node_creator->size() == 0);	// node_creatorは空でなければならない．


	// 追加したい場合，以下のように記述する．
	// (*node_creator)[HexapodMove::???] = std::make_unique<クラス名>(クラスのコンストラクタの引数);
	// この場合，HexapodMove::???のノードを作成するクラスは，↑ である．

	(*node_creator)[HexapodMove::kLegHierarchyChange] = std::make_unique<LegHierarchyNodeCreator>(HexapodMove::kLegUpDown);

	(*node_creator)[HexapodMove::kLegUpDown] = std::make_unique<LegUpDownNodeCreator>(
		map,
		converter_ptr_, 
		presenter_ptr_, 
		checker_ptr_,
		HexapodMove::kComUpDown
	);

	(*node_creator)[HexapodMove::kComUpDown] = std::make_unique<ComUpDownNodeCreator>(
		map, 
		converter_ptr_, 
		presenter_ptr_,
		checker_ptr_,
		HexapodMove::kComMove
	);

	(*node_creator)[HexapodMove::kComMove] = std::make_unique<ComMoveNodeCreatorHato>(
		map, 
		converter_ptr_, 
		presenter_ptr_, 
		checker_ptr_, 
		HexapodMove::kLegHierarchyChange
	);
}