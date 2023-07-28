#include "GraphTreeCreatorThread.h"
#include <boost/thread.hpp>
#include <vector>
#include <array>
#include <memory>
#include "Define.h"

//https://stackoverflow.com/questions/5416800/how-can-i-add-boost-threads-to-a-vector

EGraphSearchResult GraphTreeCreatorThread::createGraphTree(const SNode& currentNode, const MapState* const pointerMap, std::vector<SNode>& outputGraph, int& _make_node_num)
{
	//���݂̃m�[�h��e�ɂ���D
	SNode parentNode = currentNode;

	parentNode.changeParentNode();
	outputGraph.clear();				//�o�͂��錋�ʂ���ɂ���D
	outputGraph.push_back(parentNode);	//�e��ǉ�����D

	//�[��1�̃m�[�h�𐶐�����D
	std::vector<SNode> depth1Nodes;

	makeNewNodesByCurrentNode(parentNode, 0, depth1Nodes);


	//�[���ő�܂ŁC�}���`�X���b�h�Ńm�[�h�𐶐�����D
	boost::thread_group graphMakerGroup;	//�X���b�h���i�[����x�N�^�[

	const size_t depth1NodesSize = depth1Nodes.size();	//�[��1�̃m�[�h�̐�

	std::vector<std::vector<SNode>> threadOutputGraph(depth1NodesSize);	//�X���b�h���Ƃ̌��ʂ��i�[����x�N�^�[


	//�X���b�h�𐶐�����D
	for (size_t i = 0; i < depth1NodesSize; ++i)
	{
		graphMakerGroup.create_thread(boost::bind(&GraphTreeCreatorThread::makeGraphToMaxDepth, this, depth1Nodes[i], &threadOutputGraph[i]));
	}

	//�X���b�h�̏I����҂D
	graphMakerGroup.join_all();

	//�X���b�h���Ƃ̌��ʂ���������D
	for (size_t i = 0; i < depth1NodesSize; ++i)
	{
		const int parentIndex = static_cast<int>(outputGraph.size());	//�e�̃C���f�b�N�X

		for (auto& j : threadOutputGraph[i])
		{
			if (j.depth != 1)
			{
				j.parent_num = parentIndex + j.parent_num;
			}

			outputGraph.push_back(j);
		}
	}

	return EGraphSearchResult::Success;
}

void GraphTreeCreatorThread::makeNewNodesByCurrentNode(const SNode& _current_node, const int _current_num, std::vector<SNode>& _output_graph)
{
	_output_graph.clear();

	if (m_node_creator_map.count(_current_node.next_move) > 0)
	{
		m_node_creator_map.at(_current_node.next_move)->create(_current_node, _current_num, _output_graph);
		return;
	}
	else
	{
		//��`����Ă��Ȃ��Ȃ�΁C�����m�[�h�����̂܂ܒǉ�����D
		SNode _new_node = _current_node;

		_new_node.changeNextNode(_current_num, _current_node.next_move);

		_output_graph.push_back(_new_node);
	}
}

void GraphTreeCreatorThread::makeGraphToMaxDepth(const SNode& currentNode, std::vector<SNode>* outputGraph)
{
	int cnt = 0;	//�J�E���^��p��

	(*outputGraph).push_back(currentNode);	//�e��ǉ�����D

	//�J�E���^��vector�̃T�C�Y�𒴂���܂Ń��[�v����D
	while (cnt < (*outputGraph).size())
	{
		//�T���[��������Ă��Ȃ��m�[�h�ɂ̂ݏ���������D
		if ((*outputGraph)[cnt].depth < Define::GRAPH_SEARCH_DEPTH)
		{
			std::vector<SNode> resultGraph;	// _cnt�Ԗڂ̃m�[�h�̎q�m�[�h������x�N�^�[

			makeNewNodesByCurrentNode((*outputGraph)[cnt], cnt, resultGraph);		//�q�m�[�h�𐶐�����D

			for (const auto& i : resultGraph)
			{
				(*outputGraph).push_back(i);		//�q�m�[�h�����ʂɒǉ�����D
			}
		}

		cnt++;	//�J�E���^��i�߂�D
	}
}
