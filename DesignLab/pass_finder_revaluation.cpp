#include "pass_finder_revaluation.h"

#include "cassert_define.h"
#include "cmdio_util.h"
#include "graph_search_const.h"
#include "map_state.h"


namespace dlio = designlab::cmdio;


PassFinderRevaluation::PassFinderRevaluation(
	std::unique_ptr<IGraphTreeCreator>&& graph_tree_creator_ptr,
	std::unique_ptr<IGraphTreeCreator>&& graph_tree_creator_revaluation_ptr,
	std::unique_ptr<IGraphSearcher>&& graph_searcher_ptr,
	const std::shared_ptr<const AbstractHexapodStateCalculator>& hexapod_state_calculator_ptr
) : 
graph_tree_creator_ptr_(std::move(graph_tree_creator_ptr)),
	graph_tree_creator_revaluation_ptr_(std::move(graph_tree_creator_revaluation_ptr)),
	graph_searcher_ptr_(std::move(graph_searcher_ptr)),
	hexapod_state_calculator_ptr_(hexapod_state_calculator_ptr)
{
}

GraphSearchResult PassFinderRevaluation::GetNextNodebyGraphSearch(const RobotStateNode& current_node, const MapState& map_state, const TargetRobotState& target, RobotStateNode* output_node)
{
	assert(output_node != nullptr);	// output_node��nullptr�łȂ�


	dlio::Output("PassFinderBasic::GetNextNodebyGraphSearch�D\n�܂��͏���������D(�}�b�v�𕪊�����)\n", OutputDetail::kDebug);

	//�������^�[���D2�̃N���X�����݂��Ȃ��Ȃ�΁C�����ɏI������Dassert�ł��悩��������
	if (!graph_tree_creator_ptr_) { return GraphSearchResult::kFailureByInitializationFailed; }
	if (!graph_tree_creator_revaluation_ptr_) { return GraphSearchResult::kFailureByInitializationFailed; }
	if (!graph_searcher_ptr_) { return GraphSearchResult::kFailureByInitializationFailed; }

	DevideMapState devide_map;
	devide_map.Init(map_state, current_node.global_center_of_mass);

	graph_tree_creator_ptr_->Init(devide_map);
	graph_tree_creator_revaluation_ptr_->Init(devide_map);

	graph_tree_.clear();

	{
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

		if (IsVaildNode(current_node, (*output_node))) 
		{
			dlio::Output("�O���t�؂̕]���I���D�O���t�T���ɐ�������", OutputDetail::kDebug);

			return GraphSearchResult::kSuccess;
		}
	}

	dlio::Output("�r�O�������Ɏ��s����", OutputDetail::kDebug);

	graph_tree_.clear();

	{
		// �O���t�T�������邽�߂́C���e�p�^�[���O���t�𐶐�����
		dlio::Output("�ĕ]�����s��", OutputDetail::kDebug);

		RobotStateNode parent_node = current_node;
		parent_node.ChangeParentNode();

		GraphSearchResult result = graph_tree_creator_revaluation_ptr_->CreateGraphTree(parent_node, GraphSearchConst::kMaxDepth, &graph_tree_);

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

		if (IsVaildNode(current_node, (*output_node))) 
		{
			dlio::Output("�O���t�؂̕]���I���D�O���t�T���ɐ�������", OutputDetail::kDebug);

			return GraphSearchResult::kSuccess;
		}
	}

	dlio::Output("�r�O�������Ɏ��s����", OutputDetail::kDebug);

	return GraphSearchResult::kFailureByLegPathGenerationError;
}

int PassFinderRevaluation::GetMadeNodeNum() const
{
	return static_cast<int>(graph_tree_.size());
}

void PassFinderRevaluation::GetGraphTree(std::vector<RobotStateNode>* output_graph) const
{
	assert(output_graph != nullptr);
	assert((*output_graph).size() == 0);

	(*output_graph) = graph_tree_;
}

bool PassFinderRevaluation::IsVaildNode(const RobotStateNode& current_node, const RobotStateNode& next_node) const
{
	//�t�^���w�ŊԐڊp�x���v�Z����
	//�Ԑڊp�x���͈͓��Ɏ��܂��Ă��邩���m�F����
	std::array<HexapodJointState, HexapodConst::kLegNum> joint_state;

	//���݂̃m�[�h�̊Ԑڊp�x���v�Z����
	hexapod_state_calculator_ptr_->CalculateAllJointState(current_node, &joint_state);

	//�������������m���߂�
	if (!hexapod_state_calculator_ptr_->IsVaildJointState(current_node, joint_state))
	{
		return false;
	}

	//���̃m�[�h�̊Ԑڊp�x���v�Z����
	hexapod_state_calculator_ptr_->CalculateAllJointState(next_node, &joint_state);

	//�������������m���߂�
	if (!hexapod_state_calculator_ptr_->IsVaildJointState(next_node, joint_state))
	{
		return false;
	}

	std::vector<RobotStateNode> interpolated_node;

	interpolated_node_creator_.CreateInterpolatedNode(current_node, next_node, &interpolated_node);

	for (const auto &i : interpolated_node)
	{
		hexapod_state_calculator_ptr_->CalculateAllJointState(i, &joint_state);

		//�������������m���߂�
		if (!hexapod_state_calculator_ptr_->IsVaildJointState(i, joint_state))
		{
			return false;
		}
	}

	return true;
}
