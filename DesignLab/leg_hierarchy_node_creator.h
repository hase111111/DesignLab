#pragma once

#include <vector>

#include "discrete_leg_pos.h"
#include "interface_node_creator.h"



//! @class LegHierarchyNodeCreator
//! @date 2023/08/12
//! @author 長谷川
//! @brief 脚の階層構造を作るためのクラス．
class LegHierarchyNodeCreator final : public INodeCreator
{
public:

	LegHierarchyNodeCreator(const MapState* const p_map, std::shared_ptr<AbstractHexapodStateCalculator> calc, const EHexapodMove next_move);
	~LegHierarchyNodeCreator() = default;

	void create(const SNode& current_node, const int current_node_index, std::vector<SNode>* output_graph) override;

private:


	// 1脚が遊脚しているとき，その脚の状態を別の状態に変更する．
	void create1LegLifted(const SNode& current_node, const int current_node_index, std::vector<SNode>* output_graph);

	// 2脚が遊脚しているとき，その脚の状態を別の状態に変更する．
	void create2LegLifted(const SNode& current_node, const int current_node_index, std::vector<SNode>* output_graph);

	// 3脚が遊脚しているとき，その脚の状態を別の状態に変更する．
	void create3LegLifted(const SNode& current_node, const int current_node_index, std::vector<SNode>* output_graph);


	const std::vector<EDiscreteLegPos> discrete_leg_pos_list_;		//!< 離散化された脚位置のリスト，このリストの中の値から脚の状態を変更する．
};


//! @file leg_hierarchy_node_creator.h
//! @date 2023/08/12
//! @author 長谷川
//! @brief 脚の階層構造を作るためのクラス．
//! @n 行数 : @lineinfo
