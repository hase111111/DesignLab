//! @file leg_hierarchy_node_creator.h
//! @brief 脚の階層構造を作るためのクラス．

#ifndef DESIGNLAB_LEG_HIERARCHY_NODE_CREATOR_H_
#define DESIGNLAB_LEG_HIERARCHY_NODE_CREATOR_H_


#include "interface_node_creator.h"

#include <memory>
#include <vector>

#include "abstract_hexapod_state_calculator.h"
#include "discrete_leg_pos.h"
#include "hexapod_next_move.h"



//! @class LegHierarchyNodeCreator
//! @brief 脚の階層構造を作るためのクラス．
class LegHierarchyNodeCreator final : public INodeCreator
{
public:

	LegHierarchyNodeCreator(EHexapodMove next_move);
	~LegHierarchyNodeCreator() = default;

	void Create(const SNode& current_node, int current_node_index, std::vector<SNode>* output_graph) override;

private:


	// 1脚が遊脚しているとき，その脚の状態を別の状態に変更する．
	void create1LegLifted(const SNode& current_node, int current_node_index, std::vector<SNode>* output_graph);

	// 2脚が遊脚しているとき，その脚の状態を別の状態に変更する．
	void create2LegLifted(const SNode& current_node, int current_node_index, std::vector<SNode>* output_graph);

	// 3脚が遊脚しているとき，その脚の状態を別の状態に変更する．
	void create3LegLifted(const SNode& current_node, int current_node_index, std::vector<SNode>* output_graph);


	const EHexapodMove next_move_;									

	const std::vector<DiscreteLegPos> discrete_leg_pos_list_;		//!< 離散化された脚位置のリスト，このリストの中の値から脚の状態を変更する．
};



#endif // !DESIGNLAB_LEG_HIERARCHY_NODE_CREATOR_H_