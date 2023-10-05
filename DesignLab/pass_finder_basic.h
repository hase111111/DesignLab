//! @file pass_finder_basic.h
//! @brief ���ʂɃO���t�T�����s���C���e�p�^�[���������s���N���X

#ifndef DESIGNLAB_PASS_FINDER_BASIC_H_
#define DESIGNLAB_PASS_FINDER_BASIC_H_


#include <memory>
#include <vector>

#include "interface_pass_finder.h"
#include "interface_graph_searcher.h"
#include "interface_graph_tree_creator.h"
#include "robot_state_node.h"


//! @class PassFinderBasic
//! @brief ���ʂɃO���t�T�����s���C���e�p�^�[���������s���N���X
class PassFinderBasic final : public IPassFinder
{
public:

	//! @param[in] graph_tree_creator �O���t�T�����s���؍\���̃O���t���쐬����N���X�Dunique_ptr�œn��
	//! @param[in] graph_searcher �O���t�T�����s���N���X�Dunique_ptr�œn��
	//! @param[in] calculator_ptr ���{�b�g�̏�Ԃ��v�Z����N���X�Dshared_ptr�œn��
	PassFinderBasic(
		std::unique_ptr<IGraphTreeCreator>&& graph_tree_creator_ptr, 
		std::unique_ptr<IGraphSearcher>&& graph_searcher_ptr,
		const std::shared_ptr<const AbstractHexapodStateCalculator> calculator_ptr
	);

	~PassFinderBasic() = default;


	GraphSearchResult GetNextNodebyGraphSearch(const RobotStateNode& current_node, const MapState& map_ref, const STarget& target, RobotStateNode* output_node) override;

	int GetMadeNodeNum() const;

	void GetGraphTree(std::vector<RobotStateNode>* output_graph) const;

private:

	std::vector<RobotStateNode> graph_tree_;	//!< �O���t�T���̌��ʓ���ꂽ�؍\���̃O���t

	const std::unique_ptr<IGraphTreeCreator> graph_tree_creator_ptr_;	//!< �O���t�T�����s���؍\���̃O���t���쐬����N���X

	const std::unique_ptr<IGraphSearcher> graph_searcher_ptr_;		//!< �O���t�T�����s���N���X


	const std::shared_ptr<const AbstractHexapodStateCalculator> calculator_ptr_;	//!< ���{�b�g�̏�Ԃ��v�Z����N���X
};


#endif  // DESIGNLAB_PASS_FINDER_BASIC_H_