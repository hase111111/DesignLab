#pragma once
#include "MapState.h"
#include "Node.h"

class LegHierarchyNodeCreator final
{
public:

	void create(const SNode& _current_node, const int _current_num, std::vector<SNode>& _output_graph);

private:
	//�S�ď�̊֐��ɂ܂Ƃ߂�Ƃ�������̂ŁC�ȉ��̊֐��ɏ����𕪂��Ă����D

	// 1�r���V�r���Ă���Ƃ��C���̋r�̏�Ԃ�ʂ̏�ԂɕύX����D
	void create1LegLifted(const SNode& _current_node, const int _current_num, std::vector<SNode>& _output_graph);

	// 2�r���V�r���Ă���Ƃ��C���̋r�̏�Ԃ�ʂ̏�ԂɕύX����D
	void create2LegLifted(const SNode& _current_node, const int _current_num, std::vector<SNode>& _output_graph);	
	
	// 3�r���V�r���Ă���Ƃ��C���̋r�̏�Ԃ�ʂ̏�ԂɕύX����D
	void create3LegLifted(const SNode& _current_node, const int _current_num, std::vector<SNode>& _output_graph);

	const EHexapodMove m_next_move = EHexapodMove::LEG_UP_DOWN;
};
