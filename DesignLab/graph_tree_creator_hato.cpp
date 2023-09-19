#include "graph_tree_creator_hato.h"

#include <iostream>

#include "graph_search_const.h"



GraphTreeCreatorHato::GraphTreeCreatorHato(std::map<EHexapodMove, std::unique_ptr<INodeCreator>>& _map) : IGraphTreeCreator(_map)
{
}


EGraphSearchResult GraphTreeCreatorHato::createGraphTree(const SNode& current_node, const MapState_Old* const p_map, std::vector<SNode>* output_graph)
{
	(*output_graph).clear();					//�o�͂��錋�ʂ���ɂ���D
	(*output_graph).emplace_back(current_node);	//�e��ǉ�����D


	int cnt = 0;	//�J�E���^��p��

	//�J�E���^��vector�̃T�C�Y�𒴂���܂Ń��[�v����D
	while (cnt < (*output_graph).size())
	{
		//�T���[��������Ă��Ȃ��m�[�h�ɂ̂ݏ���������D
		if ((*output_graph)[cnt].depth < getMaxDepth())
		{
			std::vector<SNode> res_vec;	// _cnt�Ԗڂ̃m�[�h�̎q�m�[�h������x�N�^�[

			makeNewNodesByCurrentNode((*output_graph)[cnt], cnt, &res_vec);		//�q�m�[�h�𐶐�����D

			for (const auto& i : res_vec)
			{
				(*output_graph).emplace_back(i);		//�q�m�[�h�����ʂɒǉ�����D
			}
		}

		cnt++;	//�J�E���^��i�߂�D
	}


	//�m�[�h��������𒴂��Ă��Ȃ����m�F����D
	int make_node_num = static_cast<int>((*output_graph).size());

	if (GraphSearchConst::MAX_NODE_NUM < make_node_num)
	{
		return EGraphSearchResult::FailureByNodeLimitExceeded;
	}

	return EGraphSearchResult::Success;
}


void GraphTreeCreatorHato::makeNewNodesByCurrentNode(const SNode& current_node, const int current_num, std::vector<SNode>* output_graph)
{
	(*output_graph).clear();

	if (m_node_creator_map.count(current_node.next_move) > 0)
	{
		m_node_creator_map[current_node.next_move]->create(current_node, current_num, output_graph);

		return;
	}
	else
	{
		//��`����Ă��Ȃ��Ȃ�΁C�����m�[�h�����̂܂ܒǉ�����D
		SNode new_node = current_node;

		new_node.changeNextNode(current_num, current_node.next_move);

		(*output_graph).emplace_back(new_node);
	}
}
