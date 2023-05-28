#include "GraphSearcherRandom.h"
#include <algorithm>
#include <random>

bool GraphSearcherRandom::searchGraphTree(const std::vector<SNode>& _graph, const STarget& _target, SNode& _output_result)
{
	//�����ɁC�O���t��]������ _output_result �ɒl�����鏈���������D���̃N���X�̓T���v���Ȃ̂Ńe�L�g�[�ɑI�т܂��D

	if (_graph.empty() == true) { return false; }	//�O���t���Ȃ��Ȃ�false	


	//�O���t�̒���S�ĒT������D
	std::vector<SNode> _depth1_node;

	size_t _graph_size = _graph.size();
	for (size_t i = 0; i < _graph_size; i++)
	{
		// �[��1�C���̓���̒�����C��ԍŏ��Ɍ����������̂��L�^����D
		if (_graph.at(i).depth == 1)
		{
			_depth1_node.push_back(_graph.at(i));
		}
	}

	if (_depth1_node.empty()) return false;

	// �V���b�t��
	std::random_device seed_gen;
	std::mt19937 engine(seed_gen());
	std::shuffle(_depth1_node.begin(), _depth1_node.end(), engine);

	_output_result = _depth1_node.back();
	return true;
}
