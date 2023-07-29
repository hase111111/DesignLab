#include "GraphTreeCreatorThread.h"
#include <vector>
#include <memory>
#include "Define.h"

//https://stackoverflow.com/questions/5416800/how-can-i-add-boost-threads-to-a-vector

EGraphSearchResult GraphTreeCreatorThread::createGraphTree(const SNode& current_node, const MapState* const p_map, std::vector<SNode>* output_graph, int* _make_node_num)
{
	//if (DO_DEBUG_PRINT) { std::cout << "\nGraphTreeCreatorThread : �����J�n\n"; }

	////���݂̃m�[�h��e�ɂ���D
	//SNode parent_node = current_node;

	//parent_node.changeParentNode();
	//output_graph.clear();				//�o�͂��錋�ʂ���ɂ���D
	//output_graph.push_back(parent_node);	//�e��ǉ�����D

	////�[��1�̃m�[�h�𐶐�����D
	//std::vector<SNode> depth1_nodes;

	//makeNewNodesByCurrentNode(parent_node, 0, &depth1_nodes);


	////�[���ő�܂ŁC�}���`�X���b�h�Ńm�[�h�𐶐�����D
	//boost::thread_group graph_make_threads;	//�X���b�h���i�[����x�N�^�[

	//const size_t kDepth1NodesSize = depth1_nodes.size();	//�[��1�̃m�[�h�̐�

	//std::vector<std::vector<SNode>> thread_output_graph(kDepth1NodesSize);	//�X���b�h���Ƃ̌��ʂ��i�[����x�N�^�[

	////�X���b�h�𐶐�����D
	//for (size_t i = 0; i < kDepth1NodesSize; ++i)
	//{
	//	graph_make_threads.create_thread(boost::bind(&GraphTreeCreatorThread::makeGraphToMaxDepth, this, depth1_nodes[i], &thread_output_graph[i]));
	//}

	////�X���b�h�̏I����҂D
	//graph_make_threads.join_all();

	//if (DO_DEBUG_PRINT) { std::cout << "join all : �X���b�h�̏I����҂��܂����D\n"; }

	////�X���b�h���Ƃ̌��ʂ���������D
	//for (size_t i = 0; i < kDepth1NodesSize; ++i)
	//{
	//	const int kParentIndex = static_cast<int>(output_graph.size());	//�e�̃C���f�b�N�X

	//	for (auto j : thread_output_graph[i])
	//	{
	//		if (j.depth != 1)
	//		{
	//			j.parent_num += kParentIndex;
	//		}

	//		output_graph.emplace_back(j);
	//	}
	//}

	//if (DO_DEBUG_PRINT) { std::cout << "�X���b�h���Ƃ̌��ʂ��������܂����D\n\n"; }

	return EGraphSearchResult::Success;
}

void GraphTreeCreatorThread::makeNewNodesByCurrentNode(const SNode& current_node, const int current_node_index, std::vector<SNode>* output_graph) const
{
	(*output_graph).clear();

	if (m_node_creator_map.count(current_node.next_move) > 0)
	{
		m_node_creator_map.at(current_node.next_move)->create(current_node, current_node_index, output_graph);
		return;
	}
	else
	{
		//��`����Ă��Ȃ��Ȃ�΁C�����m�[�h�����̂܂ܒǉ�����D
		SNode new_node = current_node;

		new_node.changeNextNode(current_node_index, current_node.next_move);

		(*output_graph).emplace_back(new_node);
	}
}