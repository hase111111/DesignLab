#pragma once

#include "interface_node_creator.h"


//! @class LegHierarchyNodeCreator
//! @date 2023/08/12
//! @author ���J��
//! @brief �r�̊K�w�\������邽�߂̃N���X�D
class LegHierarchyNodeCreator final : public INodeCreator
{
public:

	LegHierarchyNodeCreator(const MapState* const p_map, const EHexapodMove next_move);
	~LegHierarchyNodeCreator();

	void create(const SNode& current_node, const int current_node_index, std::vector<SNode>* output_graph) override;

private:


	// 1�r���V�r���Ă���Ƃ��C���̋r�̏�Ԃ�ʂ̏�ԂɕύX����D
	void create1LegLifted(const SNode& current_node, const int current_node_index, std::vector<SNode>* output_graph);

	// 2�r���V�r���Ă���Ƃ��C���̋r�̏�Ԃ�ʂ̏�ԂɕύX����D
	void create2LegLifted(const SNode& current_node, const int current_node_index, std::vector<SNode>* output_graph);

	// 3�r���V�r���Ă���Ƃ��C���̋r�̏�Ԃ�ʂ̏�ԂɕύX����D
	void create3LegLifted(const SNode& current_node, const int current_node_index, std::vector<SNode>* output_graph);
};


//! @file leg_hierarchy_node_creator.h
//! @date 2023/08/12
//! @author ���J��
//! @brief �r�̊K�w�\������邽�߂̃N���X�D
//! @n �s�� : @lineinfo
