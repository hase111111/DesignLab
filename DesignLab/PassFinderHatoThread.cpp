#include "PassFinderHatoThread.h"
#include <iostream>
#include <boost/thread.hpp>
#include "GraphSearchConst.h"


PassFinderHatoThread::PassFinderHatoThread(std::unique_ptr<AbstractPassFinderFactory>&& _factory) : IPassFinder(std::move(_factory))
{
	if (GraphSearchConst::DO_DEBUG_PRINT) { std::cout << "\n[PassFinder] PassFinderHatoThread : �R���X�g���N�^���Ă΂ꂽ\n"; }
};

EGraphSearchResult PassFinderHatoThread::getNextNodebyGraphSearch(const SNode& current_node, const MapState* const p_map, const STarget& target, SNode& output_node)
{
	if (GraphSearchConst::DO_DEBUG_PRINT) { std::cout << "\n[PassFinder] PassFinderHatoThread : getNextNodebyGraphSearch() �O���t�T���J�n�C�܂��͏���������\n"; }

	//�����������D
	if (!mp_factory) { return EGraphSearchResult::FailureByInitializationFailed; }

	mp_factory->createGraphTreeCreator(p_map, mp_tree_creator);
	mp_factory->createGraphSearcher(mp_searcher);

	//�������^�[���D2�̃N���X�̏������Ɏ��s�����Ȃ�΁C�����ɏI������D
	if (!mp_tree_creator) { return EGraphSearchResult::FailureByInitializationFailed; }
	if (!mp_searcher) { return EGraphSearchResult::FailureByInitializationFailed; }


	if (GraphSearchConst::DO_DEBUG_PRINT) { std::cout << "\n[PassFinder] PassFinderHatoThread : �������I���D�O���t�T�����J�n����D\n"; }

	m_graph_tree.clear();

	//�܂��̓O���t���쐬����D_graph_tree �ϐ��Ɍ��ʂ��Q�Ɠn�������D
	{
		//�[��1�܂ł̃O���t���쐬����D
		SNode parent_node = current_node;
		parent_node.changeParentNode();
		m_graph_tree.emplace_back(parent_node);

		mp_tree_creator->setMaxDepth(1);

		std::vector<SNode> depth1_node;

		EGraphSearchResult result = mp_tree_creator->createGraphTree(parent_node, p_map, &depth1_node, &m_made_node_num);
		if (graphSeachResultIsSuccessful(result) == false) { return result; }

		depth1_node.erase(depth1_node.begin());	//�[��0�̃m�[�h�͐e�m�[�h�Ɠ����Ȃ̂ŁC�폜����D

		if (GraphSearchConst::DO_DEBUG_PRINT)
		{
			std::cout << "\n[PassFinder] PassFinderHatoThread : �[��1�̃m�[�h��" << depth1_node.size() << "��\n";
			std::cout << "[PassFinder] PassFinderHatoThread : �X���b�h��" << depth1_node.size() << "�쐬���܂�\n\n";
		}

		//�[��1�̃m�[�h��e�ɂ��āC�����̃O���t���쐬����D
		const size_t kDepth1NodeNum = depth1_node.size();
		std::vector<std::unique_ptr<IGraphTreeCreator>> tree_creators(kDepth1NodeNum);
		std::vector<std::vector<SNode>> threads_result(kDepth1NodeNum);
		boost::thread_group tree_creator_threads;

		for (size_t i = 0; i < kDepth1NodeNum; i++)
		{
			mp_factory->createGraphTreeCreator(p_map, tree_creators[i]);
			if (!tree_creators[i]) { return EGraphSearchResult::FailureByInitializationFailed; }
			tree_creator_threads.create_thread(boost::bind(&IGraphTreeCreator::createGraphTree, tree_creators[i].get(), depth1_node[i], p_map, &threads_result[i], &m_made_node_num));
		}

		tree_creator_threads.join_all();

		if (GraphSearchConst::DO_DEBUG_PRINT)
		{
			std::cout << "\n[PassFinder] PassFinderHatoThread : �X���b�h�I��\n";

			for (size_t i = 0; i < kDepth1NodeNum; i++)
			{
				std::cout << "[PassFinder] PassFinderHatoThread : �q�m�[�h��" << threads_result[i].size() << "\n";
			}
		}

		//�X���b�h���Ƃ̌��ʂ���������D
		for (size_t i = 0; i < kDepth1NodeNum; ++i)
		{
			const int kParentIndex = static_cast<int>(m_graph_tree.size());	//�e�̃C���f�b�N�X

			for (auto j : threads_result[i])
			{
				if (j.depth != 1)
				{
					j.parent_num += kParentIndex;
				}

				m_graph_tree.emplace_back(j);
			}
		}

		if (GraphSearchConst::DO_DEBUG_PRINT) { std::cout << "\n[PassFinder] PassFinderHatoThread : ���������D�S����" << m_graph_tree.size() << "��\n"; }
	}

	if (GraphSearchConst::DO_DEBUG_PRINT) { std::cout << "\n[PassFinder] PassFinderHato : �O���t�쐬�I���D�O���t��]������D\n"; }

	//���ɃO���t��]�����āC���̓�������肷��Dput_node �ϐ��Ɍ��ʂ��Q�Ɠn�������D
	{
		EGraphSearchResult result = mp_searcher->searchGraphTree(m_graph_tree, target, &output_node);
		if (graphSeachResultIsSuccessful(result) == false) { return result; }
	}

	if (GraphSearchConst::DO_DEBUG_PRINT) { std::cout << "\n[PassFinder] PassFinderHato : �O���t�]���I���D�O���t�T�����I������D\n"; }

	return EGraphSearchResult::Success;
}
