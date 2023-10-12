#include "pass_finder_basic.h"

#include "cassert_define.h"
#include "cmdio_util.h"
#include "graph_search_const.h"
#include "map_state.h"

namespace dlio = designlab::cmdio;


PassFinderBasic::PassFinderBasic(
	std::unique_ptr<IGraphTreeCreator>&& graph_tree_creator, 
	std::unique_ptr<IGraphSearcher>&& graph_searcher,
	const std::shared_ptr<const AbstractHexapodStateCalculator> calculator_ptr
) :
	graph_tree_({}),
	graph_tree_creator_ptr_(std::move(graph_tree_creator)), 
	graph_searcher_ptr_(std::move(graph_searcher)),
	calculator_ptr_(calculator_ptr)
{
}

GraphSearchResult PassFinderBasic::GetNextNodebyGraphSearch(const RobotStateNode& current_node, const MapState& map_ref, const TargetRobotState& target, RobotStateNode* output_node)
{
	assert(output_node != nullptr);	// output_node��nullptr�łȂ�


	dlio::Output("PassFinderBasic::GetNextNodebyGraphSearch�D\n�܂��͏���������D(�}�b�v�𕪊�����)\n", OutputDetail::kDebug);

	//�������^�[���D2�̃N���X�����݂��Ȃ��Ȃ�΁C�����ɏI������Dassert�ł��悩��������
	if (!graph_tree_creator_ptr_) { return GraphSearchResult::kFailureByInitializationFailed; }
	if (!graph_searcher_ptr_) { return GraphSearchResult::kFailureByInitializationFailed; }

	DevideMapState devide_map;
	devide_map.Init(map_ref);

	graph_tree_creator_ptr_->Init(devide_map);

	graph_tree_.clear();

	dlio::Output("�������I���D", OutputDetail::kDebug);


	// �O���t�T�������邽�߂́C���e�p�^�[���O���t�𐶐�����
	dlio::Output("�O���t�؂��쐬����", OutputDetail::kDebug);

	RobotStateNode parent_node = current_node;
	parent_node.ChangeParentNode();

	GraphSearchResult result = graph_tree_creator_ptr_->CreateGraphTree(parent_node, GraphSearchConst::kMaxDepth, &graph_tree_);

	if (result != GraphSearchResult::kSuccess)
	{
		dlio::Output("�O���t�؂̍쐬�Ɏ��s�D", OutputDetail::kDebug);
		return result; 
	}

	dlio::Output("�O���t�؂̍쐬�I���D", OutputDetail::kDebug);
	dlio::Output("�O���t�̃T�C�Y" + std::to_string(graph_tree_.size()), OutputDetail::kDebug);


	// �O���t�T�����s��
	dlio::Output("�O���t�؂�]������", OutputDetail::kDebug);

	result = graph_searcher_ptr_->SearchGraphTree(graph_tree_, target, output_node);

	if (result != GraphSearchResult::kSuccess)
	{
		dlio::Output("�O���t�؂̕]���Ɏ��s�D", OutputDetail::kDebug);
		return result; 
	}

	dlio::Output("�O���t�؂̕]���I���D�O���t�T���ɐ�������", OutputDetail::kDebug);

	return GraphSearchResult::kSuccess;
}

int PassFinderBasic::GetMadeNodeNum() const
{
	return static_cast<int>(graph_tree_.size());
}

void PassFinderBasic::GetGraphTree(std::vector<RobotStateNode>* output_graph) const
{
	assert(output_graph != nullptr);
	assert((*output_graph).size() == 0);

	(*output_graph) = graph_tree_;
}