#pragma once

#include "interface_graph_tree_creator.h"


//! @class GraphTreeCreatorHato
//! @date 2023/7/23
//! @author ���J��
//! @brief �g������̃O���t���쐬����v���O�������ڐA��������
//! @details ���Ƃ��Ƃ̃v���O�����ōs��ꂽ�����̒��ŃR�����g�A�E�g����Ă������̂͂��ׂč폜�����̂ŁC�m�肽����Ήߋ��̃v���O�������Q�Ƃ��邱�ƁD
class GraphTreeCreatorHato final : public IGraphTreeCreator
{
public:
	GraphTreeCreatorHato(std::map<EHexapodMove, std::unique_ptr<INodeCreator>>& map);
	~GraphTreeCreatorHato() = default;

	EGraphSearchResult createGraphTree(const SNode& current_node, const MapState_Old* const p_map, std::vector<SNode>* output_graph) override;

private:

	// out_put_graph�̒l�����Z�b�g���Ă���C_current_node�̎q�m�[�h�𐶐����āCoutput_graph�ɑ������D
	void makeNewNodesByCurrentNode(const SNode& current_node, const int current_num, std::vector<SNode>* output_graph);

};



//! @file graph_tree_creator_hato.h 
//! @date 2023/7/23
//! @author ���J��
//! @brief �g������̃O���t���쐬����v���O�������ڐA�����N���X�̎���
//! @n �s�� : @lineinfo
