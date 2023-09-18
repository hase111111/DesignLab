#include "pass_finder_hato_thread.h"

#include <iostream>
#include <memory>

#include <boost/thread.hpp>

#include "graph_search_const.h"
#include "designlab_cmdio.h"
#include "graph_tree_creator_hato.h"
#include "graph_searcher_hato.h"
#include "leg_up_down_node_creator.h"
#include "leg_hierarchy_node_creator.h"
#include "com_up_down_node_creator.h"
#include "com_move_node_creator_hato.h"


PassFinderHatoThread::PassFinderHatoThread(const std::shared_ptr<const AbstractHexapodStateCalculator> calculator_ptr) :
	calculator_ptr_(calculator_ptr)
{
}

EGraphSearchResult PassFinderHatoThread::getNextNodebyGraphSearch(const SNode& current_node, const MapState* const p_map, const STarget& target, SNode& output_node)
{
	if (GraphSearchConst::DO_DEBUG_PRINT) { std::cout << "\n[PassFinder] PassFinderHatoThread : getNextNodebyGraphSearch() �O���t�T���J�n�C�܂��͏���������\n"; }

	//�����������D
	std::unique_ptr<IGraphTreeCreator> graph_tree_creator = createGraphTreeCreator(p_map, calculator_ptr_);	//!< �O���t�؂̍쐬�N���X
	std::unique_ptr<AbstractGraphSearcher> graph_searcher = createGraphSearcher(calculator_ptr_);			//!< �O���t�T���N���X

	//�������^�[���D2�̃N���X�̏������Ɏ��s�����Ȃ�΁C�����ɏI������D
	if (!graph_tree_creator) { return EGraphSearchResult::FailureByInitializationFailed; }
	if (!graph_searcher) { return EGraphSearchResult::FailureByInitializationFailed; }


	if (GraphSearchConst::DO_DEBUG_PRINT) { std::cout << "\n[PassFinder] PassFinderHatoThread : �������I���D�O���t�T�����J�n����D\n"; }

	m_graph_tree.clear();

	//�܂��̓O���t���쐬����D_graph_tree �ϐ��Ɍ��ʂ��Q�Ɠn�������D
	{
		//�[��1�܂ł̃O���t���쐬����D
		SNode parent_node = current_node;
		parent_node.changeParentNode();
		m_graph_tree.emplace_back(parent_node);

		graph_tree_creator->setMaxDepth(1);

		std::vector<SNode> depth1_node;

		EGraphSearchResult result = graph_tree_creator->createGraphTree(parent_node, p_map, &depth1_node);

		if (!graphSeachResultIsSuccessful(result)) { return result; }

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
			tree_creators[i] = createGraphTreeCreator(p_map, calculator_ptr_);

			if (!tree_creators[i]) { return EGraphSearchResult::FailureByInitializationFailed; }

			tree_creator_threads.create_thread(boost::bind(&IGraphTreeCreator::createGraphTree, tree_creators[i].get(), depth1_node[i], p_map, &threads_result[i]));
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
		EGraphSearchResult result = graph_searcher->searchGraphTree(m_graph_tree, target, &output_node);
		if (!graphSeachResultIsSuccessful(result)) { return result; }
	}

	if (GraphSearchConst::DO_DEBUG_PRINT) { std::cout << "\n[PassFinder] PassFinderHato : �O���t�]���I���D�O���t�T�����I������D\n"; }

	return EGraphSearchResult::Success;
}

std::unique_ptr<IGraphTreeCreator> PassFinderHatoThread::createGraphTreeCreator(const MapState* const map, const std::shared_ptr<const AbstractHexapodStateCalculator>& calculator_ptr_)
{
	//�؂��쐬����N���X�̃}�b�v���쐬�D
	std::map<EHexapodMove, std::unique_ptr<INodeCreator>> node_creator_map;
	node_creator_map.emplace(EHexapodMove::LEG_HIERARCHY_CHANGE, std::make_unique<LegHierarchyNodeCreator>(map, calculator_ptr_, EHexapodMove::LEG_UP_DOWN));
	node_creator_map.emplace(EHexapodMove::LEG_UP_DOWN, std::make_unique<LegUpDownNodeCreator>(map, calculator_ptr_, EHexapodMove::COM_UP_DOWN));
	node_creator_map.emplace(EHexapodMove::COM_UP_DOWN, std::make_unique<ComUpDownNodeCreator>(map, calculator_ptr_, EHexapodMove::COM_MOVE));
	node_creator_map.emplace(EHexapodMove::COM_MOVE, std::make_unique<ComMoveNodeCreatorHato>(map, calculator_ptr_, EHexapodMove::LEG_HIERARCHY_CHANGE));

	//�؂��쐬����N���X�ƁC�؂�T������N���X���쐬�D
	std::unique_ptr<IGraphTreeCreator> p_creator = std::make_unique<GraphTreeCreatorHato>(node_creator_map);

	//���������N���X��Ԃ�
	return std::move(p_creator);
}

std::unique_ptr<AbstractGraphSearcher> PassFinderHatoThread::createGraphSearcher(const std::shared_ptr<const AbstractHexapodStateCalculator>& calculator_ptr_)
{
	std::unique_ptr<AbstractGraphSearcher> p_searcher = std::make_unique<GraphSearcherHato>(calculator_ptr_);
	return std::move(p_searcher);
}
