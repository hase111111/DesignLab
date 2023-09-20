//! @file graph_tree_creator_hato.h 
//! @brief �g������̃O���t���쐬����v���O�������ڐA�����N���X�̎���

#ifndef DESIGNLAB_GRAPH_TREE_CREATOR_HATO_H_
#define DESIGNLAB_GRAPH_TREE_CREATOR_HATO_H_

#include "interface_graph_tree_creator.h"

#include <map>
#include <memory>

#include "interface_node_creator.h"


//! @class GraphTreeCreatorHato
//! @brief �g������̃O���t���쐬����v���O�������ڐA��������
//! @details ���Ƃ��Ƃ̃v���O�����ōs��ꂽ�����̒��ŃR�����g�A�E�g����Ă������̂͂��ׂč폜�����̂ŁC
//! @n �m�肽����Ήߋ��̃v���O�������Q�Ƃ��邱�ƁD
class GraphTreeCreatorHato final : public IGraphTreeCreator
{
public:
	GraphTreeCreatorHato(std::map<EHexapodMove, std::unique_ptr<INodeCreator>>& map);
	~GraphTreeCreatorHato() = default;

	EGraphSearchResult CreateGraphTree(const SNode& current_node, int max_depth, std::vector<SNode>* output_graph) override;

private:

	// out_put_graph�̒l�����Z�b�g���Ă���C_current_node�̎q�m�[�h�𐶐����āCoutput_graph�ɑ������D
	void makeNewNodesByCurrentNode(const SNode& current_node, const int current_num, std::vector<SNode>* output_graph);


	std::map<EHexapodMove, std::unique_ptr<INodeCreator>> m_node_creator_map;	//!< �m�[�h�����N���X�̃}�b�v�D
};


#endif	//DESIGNLAB_GRAPH_TREE_CREATOR_HATO_H_