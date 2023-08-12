#pragma once
#include "InterfaceGraphTreeCreator.h"

//! @class GraphTreeCreatorHato
//! @brief �g������̃O���t���쐬����v���O�������ڐA��������
//! @details ���Ƃ��Ƃ̃v���O�����ōs��ꂽ�����̒��ŃR�����g�A�E�g����Ă������̂͂��ׂč폜�����̂ŁC�m�肽����Ήߋ��̃v���O�������Q�Ƃ��邱�ƁD
//! @date 2023/7/29
//! @author ���J��
class GraphTreeCreatorThread final : public IGraphTreeCreator
{
public:
	GraphTreeCreatorThread(std::map<EHexapodMove, std::unique_ptr<INodeCreator>>& map) : IGraphTreeCreator(map) {};
	~GraphTreeCreatorThread() = default;

	EGraphSearchResult createGraphTree(const SNode& current_node, const MapState* const p_map, std::vector<SNode>* output_graph, int* make_node_num) override;

private:

	//_out_put_graph�̒l�����Z�b�g���Ă���C_current_node�̎q�m�[�h�𐶐����āC_output_graph�ɑ������D
	void makeNewNodesByCurrentNode(const SNode& current_node, const int current_num, std::vector<SNode>* output_graph) const;

	static constexpr bool DO_DEBUG_PRINT = true;
};