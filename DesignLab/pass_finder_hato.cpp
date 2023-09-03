#include "pass_finder_hato.h"

#include "graph_search_const.h"


PassFinderHato::PassFinderHato(std::unique_ptr<AbstractPassFinderFactory>&& factory) : AbstractPassFinder(std::move(factory))
{
	if (GraphSearchConst::DO_DEBUG_PRINT) { std::cout << "\n[PassFinder] PassFinderHato : �R���X�g���N�^���Ă΂ꂽ�D\n"; }
};

PassFinderHato::~PassFinderHato()
{
	if (GraphSearchConst::DO_DEBUG_PRINT) { std::cout << "\n[PassFinder] PassFinderHato : �f�X�g���N�^���Ă΂ꂽ�D\n"; }
};

EGraphSearchResult PassFinderHato::getNextNodebyGraphSearch(const SNode& current_node, const MapState* const p_map, const STarget& target, SNode& output_node)
{
	if (GraphSearchConst::DO_DEBUG_PRINT) { std::cout << "\n[PassFinder] PassFinderHato : getNextNodebyGraphSearch() �O���t�T���J�n�C�܂��͏���������\n"; }

	//�����������D
	if (!mp_factory) { return EGraphSearchResult::FailureByInitializationFailed; }

	mp_factory->createGraphTreeCreator(p_map, mp_tree_creator);
	mp_factory->createGraphSearcher(mp_searcher);

	//�������^�[���D2�̃N���X�̏������Ɏ��s�����Ȃ�΁C�����ɏI������D
	if (!mp_tree_creator) { return EGraphSearchResult::FailureByInitializationFailed; }
	if (!mp_searcher) { return EGraphSearchResult::FailureByInitializationFailed; }


	if (GraphSearchConst::DO_DEBUG_PRINT) { std::cout << "\n[PassFinder] PassFinderHato : �������I���D�O���t�T�����J�n����D\n"; }

	m_graph_tree.clear();

	//�܂��̓O���t���쐬����D_graph_tree �ϐ��Ɍ��ʂ��Q�Ɠn�������D
	{
		SNode parent_node = current_node;
		parent_node.changeParentNode();

		EGraphSearchResult result = mp_tree_creator->createGraphTree(parent_node, p_map, &m_graph_tree);

		if (!graphSeachResultIsSuccessful(result)) { return result; }
	}

	if (GraphSearchConst::DO_DEBUG_PRINT) { std::cout << "\n[PassFinder] PassFinderHato : �O���t�쐬�I���D�O���t��]������D\n"; }

	//���ɃO���t��]�����āC���̓�������肷��Dput_node �ϐ��Ɍ��ʂ��Q�Ɠn�������D
	{
		EGraphSearchResult result = mp_searcher->searchGraphTree(m_graph_tree, target, &output_node);
		if (!graphSeachResultIsSuccessful(result)) { return result; }
	}

	if (GraphSearchConst::DO_DEBUG_PRINT) { std::cout << "\n[PassFinder] PassFinderHato : �O���t�]���I���D�O���t�T�����I������D\n"; }

	return EGraphSearchResult::Success;
}
