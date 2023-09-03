#include "pass_finder_hato.h"

#include "graph_search_const.h"



EGraphSearchResult PassFinderHato::getNextNodebyGraphSearch(const SNode& current_node, const MapState* const p_map, const STarget& target, SNode& output_node)
{
	if (GraphSearchConst::DO_DEBUG_PRINT) { std::cout << "\n[PassFinder] PassFinderHato : getNextNodebyGraphSearch() �O���t�T���J�n�C�܂��͏���������\n"; }

	//�����������D
	if (!mp_factory) { return EGraphSearchResult::FailureByInitializationFailed; }

	std::unique_ptr<IGraphTreeCreator> graph_tree_creator;	//!< �O���t�؂̍쐬�N���X
	std::unique_ptr<IGraphSearcher> graph_searcher;		//!< �O���t�T���N���X

	mp_factory->createGraphTreeCreator(p_map, mp_calculator, graph_tree_creator);
	mp_factory->createGraphSearcher(graph_searcher);

	m_graph_tree.clear();

	//�������^�[���D2�̃N���X�̏������Ɏ��s�����Ȃ�΁C�����ɏI������D
	if (!graph_tree_creator) { return EGraphSearchResult::FailureByInitializationFailed; }
	if (!graph_searcher) { return EGraphSearchResult::FailureByInitializationFailed; }


	if (GraphSearchConst::DO_DEBUG_PRINT) { std::cout << "\n[PassFinder] PassFinderHato : �������I���D�O���t�T�����J�n����D\n"; }



	//�܂��̓O���t���쐬����D_graph_tree �ϐ��Ɍ��ʂ��Q�Ɠn�������D
	{
		SNode parent_node = current_node;
		parent_node.changeParentNode();

		EGraphSearchResult result = graph_tree_creator->createGraphTree(parent_node, p_map, &m_graph_tree);

		if (!graphSeachResultIsSuccessful(result)) { return result; }
	}

	if (GraphSearchConst::DO_DEBUG_PRINT) { std::cout << "\n[PassFinder] PassFinderHato : �O���t�쐬�I���D�O���t��]������D\n"; }



	//���ɃO���t��]�����āC���̓�������肷��Dput_node �ϐ��Ɍ��ʂ��Q�Ɠn�������D
	{
		EGraphSearchResult result = graph_searcher->searchGraphTree(m_graph_tree, target, &output_node);
		if (!graphSeachResultIsSuccessful(result)) { return result; }
	}

	if (GraphSearchConst::DO_DEBUG_PRINT) { std::cout << "\n[PassFinder] PassFinderHato : �O���t�]���I���D�O���t�T�����I������D\n"; }



	return EGraphSearchResult::Success;
}
