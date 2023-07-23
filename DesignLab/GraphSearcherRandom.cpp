#include "GraphSearcherRandom.h"
#include"MyMath.h"


EGraphSearchResult GraphSearcherRandom::searchGraphTree(const std::vector<SNode>& _graph, const STarget& _target, SNode& _output_result)
{
	//�O���t��T�������ɁC���̓���̒����烉���_���Ɉ�I�����Ĉړ�����D

	if (_graph.empty() == true) { return EGraphSearchResult::Failure; }	//�O���t���Ȃ��Ȃ玸�s	


	//�O���t�̒���S�ĒT������D
	std::vector<SNode> _depth1_node;

	size_t _graph_size = _graph.size();

	for (size_t i = 0; i < _graph_size; i++)
	{
		// �[��1�̓����S�ċL�^����D
		if (_graph.at(i).depth == 1)
		{
			_depth1_node.push_back(_graph.at(i));
		}
	}

	if (_depth1_node.empty()) { return EGraphSearchResult::FailureByNotReachedDepth; }		//�[��1�̃m�[�h�����݂��Ȃ��Ȃ�C�I���D

	_output_result = _depth1_node.at(my_math::generateRandomNumber(0, (int)_depth1_node.size() - 1));		// �����_���Ȃ����I������D

	return EGraphSearchResult::Success;
}
