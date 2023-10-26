//! @file pass_finder_revaluation.h
//! @brief �ĕ]����@�����������N���X


#ifndef PASS_FINDER_REVALUATION_H_
#define PASS_FINDER_REVALUATION_H_


#include <memory>
#include <vector>

#include "interface_pass_finder.h"
#include "interface_graph_searcher.h"
#include "interface_graph_tree_creator.h"
#include "interpolated_node_creator.h"
#include "robot_state_node.h"


class PassFinderRevaluation final : public IPassFinder
{
public:


	PassFinderRevaluation(
		std::unique_ptr<IGraphTreeCreator>&& graph_tree_creator_ptr,
		std::unique_ptr<IGraphTreeCreator>&& graph_tree_creator_revaluation_ptr,
		std::unique_ptr<IGraphSearcher>&& graph_searcher_ptr,
		const std::shared_ptr<const AbstractHexapodStateCalculator>& hexapod_state_calculator_ptr
	);

	~PassFinderRevaluation() = default;


	GraphSearchResult GetNextNodebyGraphSearch(const RobotStateNode& current_node, const MapState& map_ref, const TargetRobotState& target, RobotStateNode* output_node) override;

	int GetMadeNodeNum() const;

	void GetGraphTree(std::vector<RobotStateNode>* output_graph) const;

private:

	std::vector<RobotStateNode> graph_tree_;	//!< �O���t�T���̌��ʓ���ꂽ�؍\���̃O���t

	const std::unique_ptr<IGraphTreeCreator> graph_tree_creator_ptr_;	//!< �O���t�T�����s���؍\���̃O���t���쐬����N���X

	const std::unique_ptr<IGraphTreeCreator> graph_tree_creator_revaluation_ptr_;	//!< �ĕ]���p�̖؍\���̃O���t���쐬����N���X

	const std::unique_ptr<IGraphSearcher> graph_searcher_ptr_;			//!< �O���t�T�����s���N���X

	const std::shared_ptr<const AbstractHexapodStateCalculator> hexapod_state_calculator_ptr_;	//!< �w�L�T�|�b�h�̏�Ԃ��v�Z����N���X

	InterpolatedNodeCreator interpolated_node_creator_;	//!< ��ԃm�[�h���쐬����N���X

	bool IsVaildNode(const RobotStateNode& current_node, const RobotStateNode& next_node) const;
};

#endif