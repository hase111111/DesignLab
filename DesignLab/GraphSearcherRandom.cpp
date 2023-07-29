#include "GraphSearcherRandom.h"
#include"MyMath.h"


EGraphSearchResult GraphSearcherRandom::searchGraphTree(const std::vector<SNode>& graph, const STarget& target, SNode* output_result)
{
	//�O���t��T�������ɁC���̓���̒����烉���_���Ɉ�I�����Ĉړ�����D

	if (graph.empty() == true) { return EGraphSearchResult::Failure; }	//�O���t���Ȃ��Ȃ玸�s	


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

	if (depth1_node.empty()) { return EGraphSearchResult::FailureByNotReachedDepth; }		//�[��1�̃m�[�h�����݂��Ȃ��Ȃ�C�I���D

	(*output_result) = depth1_node.at(my_math::generateRandomNumber(0, (int)depth1_node.size() - 1));		// �����_���Ȃ����I������D

	return EGraphSearchResult::Success;
}
