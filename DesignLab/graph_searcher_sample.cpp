//#include "graph_searcher_sample.h"
//
//
//
//EGraphSearchResult GraphSearcherSample::searchGraphTree(const std::vector<SNode>& graph, const STarget& target, SNode* output_result)
//{
//	//�����ɁC�O���t��]������ _output_result �ɒl�����鏈���������D���̃N���X�̓T���v���Ȃ̂Ńe�L�g�[�ɑI�ԁD
//
//	if (graph.empty()) { return EGraphSearchResult::Failure; }	//�O���t���Ȃ��Ȃ�false	
//
//
//	//�O���t�̒���S�ĒT������D
//	for (const auto& i : graph)
//	{
//		// �[��1�C���̓���̒�����C��ԍŏ��Ɍ����������̂����ʂƂ��ďo�͂���D
//		if (i.depth == 1)
//		{
//			//���ʂ����āC�I���D
//			(*output_result) = i;
//			return EGraphSearchResult::Success;
//		}
//	}
//
//	//������Ȃ����false
//	return EGraphSearchResult::FailureByNotReachedDepth;
//}
//
