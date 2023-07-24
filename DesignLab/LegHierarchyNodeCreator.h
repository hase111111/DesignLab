#pragma once
#include "InterfaceNodeCreator.h"

class LegHierarchyNodeCreator final : public INodeCreator
{
public:

	LegHierarchyNodeCreator(const MapState* const _p_map, const EHexapodMove _next_move) : INodeCreator(_p_map, _next_move) {};
	~LegHierarchyNodeCreator() = default;

	void create(const SNode& _current_node, const int _current_num, std::vector<SNode>& _output_graph) override;

private:
	//�S�ď�̊֐��ɂ܂Ƃ߂�Ƃ�������̂ŁC�ȉ��̊֐��ɏ����𕪂��Ă����D

	// 1�r���V�r���Ă���Ƃ��C���̋r�̏�Ԃ�ʂ̏�ԂɕύX����D
	void create1LegLifted(const SNode& _current_node, const int _current_num, std::vector<SNode>& _output_graph);

	// 2�r���V�r���Ă���Ƃ��C���̋r�̏�Ԃ�ʂ̏�ԂɕύX����D
	void create2LegLifted(const SNode& _current_node, const int _current_num, std::vector<SNode>& _output_graph);

	// 3�r���V�r���Ă���Ƃ��C���̋r�̏�Ԃ�ʂ̏�ԂɕύX����D
	void create3LegLifted(const SNode& _current_node, const int _current_num, std::vector<SNode>& _output_graph);
};

//! @file LegHierarchyNodeCreator.h
//! @brief �r�̊K�w�\������邽�߂̃N���X�D
//! @date 2023/7/24
//! @auther ���J��

//! @class LegHierarchyNodeCreator
//! @brief �r�̊K�w�\������邽�߂̃N���X�D
//! @date 2023/7/24
//! @author ���J��