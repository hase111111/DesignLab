#pragma once
#include "MapState.h"
#include "Node.h"

class LegHierarchyNodeCreator final
{
public:

	void create(const SNode& _current_node, const int _current_num, std::vector<SNode>& _output_graph);

private:
	//全て上の関数にまとめるとごちゃつくので，以下の関数に処理を分けておく．

	// 1脚が遊脚しているとき，その脚の状態を別の状態に変更する．
	void create1LegLifted(const SNode& _current_node, const int _current_num, std::vector<SNode>& _output_graph);

	// 2脚が遊脚しているとき，その脚の状態を別の状態に変更する．
	void create2LegLifted(const SNode& _current_node, const int _current_num, std::vector<SNode>& _output_graph);	
	
	// 3脚が遊脚しているとき，その脚の状態を別の状態に変更する．
	void create3LegLifted(const SNode& _current_node, const int _current_num, std::vector<SNode>& _output_graph);

	const EHexapodMove m_next_move = EHexapodMove::LEG_UP_DOWN;
};
