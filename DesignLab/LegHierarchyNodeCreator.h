#pragma once
#include "InterfaceNodeCreator.h"

class LegHierarchyNodeCreator final : public INodeCreator
{
public:

	LegHierarchyNodeCreator(const MapState* const p_map, const EHexapodMove next_move) : INodeCreator(p_map, next_move) {};
	~LegHierarchyNodeCreator() = default;

	void create(const SNode& current_node, const int current_node_index, std::vector<SNode>* output_graph) override;

private:
	//全て上の関数にまとめるとごちゃつくので，以下の関数に処理を分けておく．

	// 1脚が遊脚しているとき，その脚の状態を別の状態に変更する．
	void create1LegLifted(const SNode& current_node, const int current_node_index, std::vector<SNode>* output_graph);

	// 2脚が遊脚しているとき，その脚の状態を別の状態に変更する．
	void create2LegLifted(const SNode& current_node, const int current_node_index, std::vector<SNode>* output_graph);

	// 3脚が遊脚しているとき，その脚の状態を別の状態に変更する．
	void create3LegLifted(const SNode& current_node, const int current_node_index, std::vector<SNode>* output_graph);
};

//! @file LegHierarchyNodeCreator.h
//! @brief 脚の階層構造を作るためのクラス．
//! @date 2023/7/24
//! @auther 長谷川

//! @class LegHierarchyNodeCreator
//! @brief 脚の階層構造を作るためのクラス．
//! @date 2023/7/24
//! @author 長谷川