#include "graph_searcher_random.h"

#include "designlab_math_util.h"

namespace dlm = designlab::math_util;


GraphSearchResult GraphSearcherRandom::SearchGraphTree(const std::vector<SNode>& graph, const STarget& target, SNode* output_result)
{
	//�O���t��T�������ɁC���̓���̒����烉���_���Ɉ�I�����Ĉړ�����D

	if (graph.empty()) { return GraphSearchResult::Failure; }	//�O���t���Ȃ��Ȃ玸�s	

	//�x�����p
	STarget target_copy = target;

	//�O���t�̒���S�ĒT������D
	std::vector<SNode> depth1_node;

	size_t kGraphSize = graph.size();

	for (size_t i = 0; i < kGraphSize; i++)
	{
		// �[��1�̓����S�ċL�^����D
		if (graph[i].depth == 1)
		{
			depth1_node.emplace_back(graph[i]);
		}
	}

	if (depth1_node.empty()) { return GraphSearchResult::FailureByNotReachedDepth; }		//�[��1�̃m�[�h�����݂��Ȃ��Ȃ�C�I���D


	(*output_result) = depth1_node.at(dlm::GenerateRandomNumber(0, static_cast<int>(depth1_node.size()) - 1));		// �����_���Ȃ����I������D


	return GraphSearchResult::Success;
}
