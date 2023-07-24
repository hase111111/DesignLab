#include "PassFinderHato.h"

EGraphSearchResult PassFinderHato::getNextNodebyGraphSearch(const SNode& _current_node, const MapState* const _p_map, const STarget& _target, SNode& _output_node)
{
	//�����������D
	if (!mp_PassFinderFactory) { return EGraphSearchResult::FailureByInitializationFailed; }

	mp_PassFinderFactory->createPassFinder(mp_GraphTreeCreator, mp_GraphSearcher, _p_map);

	//�������^�[���D2�̃N���X�̏������Ɏ��s�����Ȃ�΁C�����ɏI������D
	if (!mp_GraphTreeCreator) { return EGraphSearchResult::FailureByInitializationFailed; }
	if (!mp_GraphSearcher) { return EGraphSearchResult::FailureByInitializationFailed; }


	std::vector<SNode> _graph_tree;		//�O���t�T���ɗp����O���t�D

	//�܂��̓O���t���쐬����D_graph_tree �ϐ��Ɍ��ʂ��Q�Ɠn�������D
	{
		EGraphSearchResult _result = mp_GraphTreeCreator->createGraphTree(_current_node, _p_map, _graph_tree, m_made_node_num);
		if (graphSeachResultIsSuccessful(_result) == false) { return _result; }
	}

	//���ɃO���t��]�����āC���̓�������肷��Dput_node �ϐ��Ɍ��ʂ��Q�Ɠn�������D
	{
		EGraphSearchResult _result = mp_GraphSearcher->searchGraphTree(_graph_tree, _target, _output_node);
		if (graphSeachResultIsSuccessful(_result) == false) { return _result; }
	}

	return EGraphSearchResult::Success;
}
