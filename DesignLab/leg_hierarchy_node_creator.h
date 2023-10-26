//! @file leg_hierarchy_node_creator.h
//! @brief �r�̊K�w�\������邽�߂̃N���X�D

#ifndef DESIGNLAB_LEG_HIERARCHY_NODE_CREATOR_H_
#define DESIGNLAB_LEG_HIERARCHY_NODE_CREATOR_H_


#include "interface_node_creator.h"

#include <memory>
#include <vector>

#include "abstract_hexapod_state_calculator.h"
#include "discrete_leg_pos.h"
#include "hexapod_next_move.h"



//! @class LegHierarchyNodeCreator
//! @brief �r�̊K�w�\������邽�߂̃N���X�D
class LegHierarchyNodeCreator final : public INodeCreator
{
public:

	LegHierarchyNodeCreator(HexapodMove next_move);
	~LegHierarchyNodeCreator() = default;

	void Create(const RobotStateNode& current_node, int current_node_index, std::vector<RobotStateNode>* output_graph) const override;

private:


	// 1�r���V�r���Ă���Ƃ��C���̋r�̏�Ԃ�ʂ̏�ԂɕύX����D
	void create1LegLifted(const RobotStateNode& current_node, int current_node_index, std::vector<RobotStateNode>* output_graph) const;

	// 2�r���V�r���Ă���Ƃ��C���̋r�̏�Ԃ�ʂ̏�ԂɕύX����D
	void create2LegLifted(const RobotStateNode& current_node, int current_node_index, std::vector<RobotStateNode>* output_graph) const;

	// 3�r���V�r���Ă���Ƃ��C���̋r�̏�Ԃ�ʂ̏�ԂɕύX����D
	void create3LegLifted(const RobotStateNode& current_node, int current_node_index, std::vector<RobotStateNode>* output_graph) const;


	const HexapodMove next_move_;									

	const std::vector<DiscreteLegPos> discrete_leg_pos_list_;		//!< ���U�����ꂽ�r�ʒu�̃��X�g�C���̃��X�g�̒��̒l����r�̏�Ԃ�ύX����D
};



#endif // !DESIGNLAB_LEG_HIERARCHY_NODE_CREATOR_H_