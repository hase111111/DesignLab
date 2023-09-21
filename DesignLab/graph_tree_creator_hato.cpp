#include "graph_tree_creator_hato.h"

#include <iostream>

#include "cassert_define.h"
#include "graph_search_const.h"



GraphTreeCreatorHato::GraphTreeCreatorHato(
	std::unique_ptr<INodeCreatorBuilder>&& node_creator_builder_ptr,
	const std::shared_ptr<const AbstractHexapodStateCalculator> calculator_ptr
) :
	node_creator_builder_ptr_(std::move(node_creator_builder_ptr)),
	calculator_ptr_(calculator_ptr)
{
}


void GraphTreeCreatorHato::Init(const DevideMapState& map_state)
{
	node_creator_map_.clear();

	node_creator_builder_ptr_->Build(map_state, calculator_ptr_, &node_creator_map_);
}

EGraphSearchResult GraphTreeCreatorHato::CreateGraphTree(const SNode& current_node, const int max_depth, std::vector<SNode>* output_graph)
{
	assert(output_graph != nullptr);	//nullptr�łȂ��D
	assert(output_graph->empty());		//��ł���D
	assert(current_node.depth == 0);	//�[����0�ł���D

	output_graph->emplace_back(current_node);	//�e��ǉ�����D


	int cnt = 0;	//�J�E���^��p��

	//�J�E���^��vector�̃T�C�Y�𒴂���܂Ń��[�v����D
	while (cnt < output_graph->size())
	{
		//�T���[��������Ă��Ȃ��m�[�h�ɂ̂ݏ���������D
		if ((*output_graph)[cnt].depth < max_depth)
		{
			std::vector<SNode> res_vec;	// _cnt�Ԗڂ̃m�[�h�̎q�m�[�h������x�N�^�[

			makeNewNodesByCurrentNode((*output_graph)[cnt], cnt, &res_vec);		//�q�m�[�h�𐶐�����D

			for (const auto& i : res_vec)
			{
				output_graph->emplace_back(i);		//�q�m�[�h�����ʂɒǉ�����D
			}
		}

		cnt++;	//�J�E���^��i�߂�D
	}


	//�m�[�h��������𒴂��Ă��Ȃ����m�F����D
	int make_node_num = static_cast<int>(output_graph->size());

	if (GraphSearchConst::MAX_NODE_NUM < make_node_num)
	{
		return EGraphSearchResult::FailureByNodeLimitExceeded;
	}

	return EGraphSearchResult::Success;
}


void GraphTreeCreatorHato::makeNewNodesByCurrentNode(const SNode& current_node, const int current_num, std::vector<SNode>* output_graph) const
{
	(*output_graph).clear();

	if (node_creator_map_.count(current_node.next_move) > 0)
	{
		node_creator_map_.at(current_node.next_move)->Create(current_node, current_num, output_graph);

		return;
	}
	else
	{
		assert(false);

		//��`����Ă��Ȃ��Ȃ�΁C�����m�[�h�����̂܂ܒǉ�����D
		SNode new_node = current_node;

		new_node.ChangeToNextNode(current_num, current_node.next_move);

		(*output_graph).emplace_back(new_node);
	}
}


//std::unique_ptr<IGraphTreeCreator> graph_tree_creator = createGraphTreeCreator(devide_map_, calculator_ptr_);	//!< �O���t�؂̍쐬�N���X
//std::unique_ptr<IGraphSearcher> graph_searcher = createGraphSearcher(calculator_ptr_);			//!< �O���t�T���N���X




//if (GraphSearchConst::DO_DEBUG_PRINT) { std::cout << "\n[PassFinder] PassFinderBasic : �������I���D�O���t�T�����J�n����D\n"; }

//m_graph_tree.clear();

//�܂��̓O���t���쐬����D_graph_tree �ϐ��Ɍ��ʂ��Q�Ɠn�������D
//{
//	//�[��1�܂ł̃O���t���쐬����D
//	SNode parent_node = current_node;
//	parent_node.ChangeParentNode();
//	m_graph_tree.emplace_back(parent_node);

//	std::vector<SNode> depth1_node;

//	EGraphSearchResult result = graph_tree_creator->CreateGraphTree(parent_node, 1, &depth1_node);

//	if (!graphSeachResultIsSuccessful(result)) { return result; }

//	depth1_node.erase(depth1_node.begin());	//�[��0�̃m�[�h�͐e�m�[�h�Ɠ����Ȃ̂ŁC�폜����D

//	if (GraphSearchConst::DO_DEBUG_PRINT)
//	{
//		std::cout << "\n[PassFinder] PassFinderBasic : �[��1�̃m�[�h��" << depth1_node.size() << "��\n";
//		std::cout << "[PassFinder] PassFinderBasic : �X���b�h��" << depth1_node.size() << "�쐬���܂�\n\n";
//	}

//	//�[��1�̃m�[�h��e�ɂ��āC�����̃O���t���쐬����D
//	const size_t kDepth1NodeNum = depth1_node.size();
//	std::vector<std::unique_ptr<IGraphTreeCreator>> tree_creators(kDepth1NodeNum);
//	std::vector<std::vector<SNode>> threads_result(kDepth1NodeNum);
//	boost::thread_group tree_creator_threads;

//	for (size_t i = 0; i < kDepth1NodeNum; i++)
//	{
//		tree_creators[i] = createGraphTreeCreator(devide_map_, calculator_ptr_);

//		if (!tree_creators[i]) { return EGraphSearchResult::FailureByInitializationFailed; }

//		tree_creator_threads.create_thread(boost::bind(&IGraphTreeCreator::CreateGraphTree, tree_creators[i].get(), depth1_node[i], GraphSearchConst::MAX_DEPTH, &threads_result[i]));
//	}

//	tree_creator_threads.join_all();

//	if (GraphSearchConst::DO_DEBUG_PRINT)
//	{
//		std::cout << "\n[PassFinder] PassFinderBasic : �X���b�h�I��\n";

//		for (size_t i = 0; i < kDepth1NodeNum; i++)
//		{
//			std::cout << "[PassFinder] PassFinderBasic : �q�m�[�h��" << threads_result[i].size() << "\n";
//		}
//	}

//	//�X���b�h���Ƃ̌��ʂ���������D
//	for (size_t i = 0; i < kDepth1NodeNum; ++i)
//	{
//		const int kParentIndex = static_cast<int>(m_graph_tree.size());	//�e�̃C���f�b�N�X

//		for (auto j : threads_result[i])
//		{
//			if (j.depth != 1)
//			{
//				j.parent_num += kParentIndex;
//			}

//			m_graph_tree.emplace_back(j);
//		}
//	}

//	if (GraphSearchConst::DO_DEBUG_PRINT) { std::cout << "\n[PassFinder] PassFinderBasic : ���������D�S����" << m_graph_tree.size() << "��\n"; }
//}

//if (GraphSearchConst::DO_DEBUG_PRINT) { std::cout << "\n[PassFinder] PassFinderHato : �O���t�쐬�I���D�O���t��]������D\n"; }

////���ɃO���t��]�����āC���̓�������肷��Dput_node �ϐ��Ɍ��ʂ��Q�Ɠn�������D
//{
//	EGraphSearchResult result = graph_searcher->SearchGraphTree(m_graph_tree, target, &output_node);
//	if (!graphSeachResultIsSuccessful(result)) { return result; }
//}

//if (GraphSearchConst::DO_DEBUG_PRINT) { std::cout << "\n[PassFinder] PassFinderHato : �O���t�]���I���D�O���t�T�����I������D\n"; }

//return EGraphSearchResult::Success;