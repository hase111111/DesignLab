#include "graph_tree_creator_hato.h"

#include <iostream>

#include <boost/thread.hpp>

#include "cassert_define.h"
#include "graph_search_const.h"



GraphTreeCreatorHato::GraphTreeCreatorHato(
	std::unique_ptr<INodeCreatorBuilder>&& node_creator_builder_ptr,
	const std::shared_ptr<const IHexapodCoordinateConverter>& converter_ptr,
	const std::shared_ptr<const IHexapodStatePresenter>& presenter_ptr,
	const std::shared_ptr<const IHexapodVaildChecker>& checker_ptr
	) :
	node_creator_builder_ptr_(std::move(node_creator_builder_ptr)),
	converter_ptr_(converter_ptr),
	presenter_ptr_(presenter_ptr),
	checker_ptr_(checker_ptr)
{
	//�������S��nullptr�łȂ����Ƃ��m�F����D
	assert(node_creator_builder_ptr_ != nullptr);	
	assert(converter_ptr_ != nullptr);	
	assert(presenter_ptr_ != nullptr);
	assert(checker_ptr_ != nullptr);
}


void GraphTreeCreatorHato::Init(const DevideMapState& map_state)
{
	node_creator_map_.clear();

	node_creator_builder_ptr_->Build(
		map_state, 
		converter_ptr_,
		presenter_ptr_,
		checker_ptr_,
		&node_creator_map_
	);
}

GraphSearchResult GraphTreeCreatorHato::CreateGraphTree(const RobotStateNode& current_node, const int max_depth, std::vector<RobotStateNode>* output_graph)
{
	assert(output_graph != nullptr);	//nullptr�łȂ��D
	assert(output_graph->empty());		//��ł���D
	assert(current_node.depth == 0);	//�[����0�ł���D

	// �܂��́C�[��1�܂ł̃O���t���쐬����D
	std::vector<RobotStateNode> depth1_graph;

	depth1_graph.emplace_back(current_node);	//�e��ǉ�����D

	MakeGraphTreeToMaxDepth(1, &depth1_graph);	//�[��1�܂ł̃O���t���쐬����D


	// ���ɁC�[��1�܂ł̃O���t��e�ɂ��āC�����̃O���t���쐬����D
	// �����̃X���b�h�ŕ���ɏ�������D�������邱�ƂŁC�������Ԃ�Z�k����D
	std::vector<std::vector<RobotStateNode>> threads_result(kMultiThreadNum);	//�X���b�h���Ƃ̌��ʂ��i�[����x�N�^�[
	boost::thread_group tree_creator_threads;	//�X���b�h���i�[����N���X

	int node_cnt = 0;	//�m�[�h�̃J�E���^

	for (const auto &i : depth1_graph)
	{
		if (i.depth == 1)
		{
			// �[��1�̃m�[�h���X���b�h���Ƃɕ�����D
			threads_result[node_cnt % kMultiThreadNum].emplace_back(i);

			node_cnt++;
		}
	}

	for (int i = 0; i < kMultiThreadNum; ++i)
	{
		//�X���b�h���쐬����D
		tree_creator_threads.create_thread(boost::bind(&GraphTreeCreatorHato::MakeGraphTreeToMaxDepth, this, max_depth, &threads_result[i]));
	}

	tree_creator_threads.join_all();	//�e�X���b�h�̏I����҂D
	
	//�X���b�h���Ƃ̌��ʂ���������D
	(*output_graph).emplace_back(current_node);	//�e��ǉ�����D

	for (int i = 0; i < kMultiThreadNum; ++i)
	{
		const int kParentIndex = static_cast<int>(output_graph->size());	//�e�̃C���f�b�N�X

		for (auto j : threads_result[i])
		{
			if (j.depth > 1)
			{
				j.parent_num += kParentIndex;
			}

			output_graph->emplace_back(j);
		}
	}

	//�m�[�h��������𒴂��Ă��Ȃ����m�F����D
	int make_node_num = static_cast<int>(output_graph->size());

	if (GraphSearchConst::kMaxNodeNum < make_node_num)
	{
		return GraphSearchResult::kFailureByNodeLimitExceeded;
	}

	return GraphSearchResult::kSuccess;
}


void GraphTreeCreatorHato::MakeGraphTreeToMaxDepth(int max_depth, std::vector<RobotStateNode>* current_graph) const
{
	assert(current_graph != nullptr);	//nullptr�łȂ��D

	if ((*current_graph).empty()) { return; }	//��Ȃ�Ή������Ȃ��D

	int cnt = 0;	//�J�E���^��p��

	//�J�E���^��vector�̃T�C�Y�𒴂���܂Ń��[�v����D
	while (cnt < current_graph->size())
	{
		//�T���[��������Ă��Ȃ��m�[�h�ɂ̂ݏ���������D
		if ((*current_graph)[cnt].depth < max_depth)
		{
			std::vector<RobotStateNode> res_vec;	// _cnt�Ԗڂ̃m�[�h�̎q�m�[�h������x�N�^�[

			MakeNewNodesByCurrentNode((*current_graph)[cnt], cnt, &res_vec);		//�q�m�[�h�𐶐�����D

			for (const auto& i : res_vec)
			{
				current_graph->emplace_back(i);		//�q�m�[�h�����ʂɒǉ�����D
			}
		}

		cnt++;	//�J�E���^��i�߂�D
	}
}

void GraphTreeCreatorHato::MakeNewNodesByCurrentNode(const RobotStateNode& current_node, const int current_num, std::vector<RobotStateNode>* output_graph) const
{
	assert(output_graph != nullptr);	//nullptr�łȂ��D
	assert(output_graph->empty());		//��ł���D

	if (node_creator_map_.count(current_node.next_move) > 0)
	{
		node_creator_map_.at(current_node.next_move)->Create(current_node, current_num, output_graph);

		return;
	}
	else
	{
		assert(false);	//�m�[�h�����N���X���o�^����Ă��Ȃ��D

		//assert�̉��ɏ�����ǉ����闝�R�Ƃ��ẮCassert���Ă΂�Ȃ��ꍇ(Release�r���h�̍ۂȂ�)�ɂ��ꉞ����\�ɂ��邽�߁D 

		//��`����Ă��Ȃ��Ȃ�΁C�����m�[�h�����̂܂ܒǉ�����D
		RobotStateNode new_node = current_node;

		new_node.ChangeToNextNode(current_num, current_node.next_move);

		(*output_graph).emplace_back(new_node);
	}
}