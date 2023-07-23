#include "GraphSearcherSample.h"

EGraphSearchResult GraphSearcherSample::searchGraphTree(const std::vector<SNode>& _graph, const STarget& _target, SNode& _output_result)
{
	//�����ɁC�O���t��]������ _output_result �ɒl�����鏈���������D���̃N���X�̓T���v���Ȃ̂Ńe�L�g�[�ɑI�т܂��D

	if (_graph.empty() == true) { return EGraphSearchResult::Failure; }	//�O���t���Ȃ��Ȃ�false	


	//�O���t�̒���S�ĒT������D
	for (const auto& i : _graph)
	{
		// �[��1�C���̓���̒�����C��ԍŏ��Ɍ����������̂����ʂƂ��ďo�͂���D
		if (i.depth == 1)
		{
			//���ʂ����āC�I���D
			_output_result = i;
			return EGraphSearchResult::Success;
		}
	}

	//������Ȃ����false
	return EGraphSearchResult::FailureByNotReachedDepth;
}