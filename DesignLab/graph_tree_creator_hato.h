//! @file graph_tree_creator_hato.h 
//! @brief �g������̃O���t���쐬����v���O�������ڐA�����N���X�̎���

#ifndef DESIGNLAB_GRAPH_TREE_CREATOR_HATO_H_
#define DESIGNLAB_GRAPH_TREE_CREATOR_HATO_H_

#include "interface_graph_tree_creator.h"

#include <map>
#include <memory>

#include "abstract_hexapod_state_calculator.h"
#include "interface_node_creator.h"
#include "interface_node_creator_builder.h"


//! @class GraphTreeCreatorHato
//! @brief �g������̃O���t���쐬����v���O�������ڐA��������
//! @details ���Ƃ��Ƃ̃v���O�����ōs��ꂽ�����̒��ŃR�����g�A�E�g����Ă������̂͂��ׂč폜�����̂ŁC
//! @n �m�肽����Ήߋ��̃v���O�������Q�Ƃ��邱�ƁD
class GraphTreeCreatorHato final : public IGraphTreeCreator
{
public:
	GraphTreeCreatorHato(
		std::unique_ptr<INodeCreatorBuilder>&& node_creator_builder_ptr,
		const std::shared_ptr<const AbstractHexapodStateCalculator> calculator_ptr
	);

	~GraphTreeCreatorHato() = default;


	void Init(const DevideMapState& map_state);

	GraphSearchResult CreateGraphTree(const RobotStateNode& current_node, int max_depth, std::vector<RobotStateNode>* output_graph) override;

private:

	// out_put_graph�̒l�����Z�b�g���Ă���C_current_node�̎q�m�[�h�𐶐����āCoutput_graph�ɑ������D
	void makeNewNodesByCurrentNode(const RobotStateNode& current_node, const int current_num, std::vector<RobotStateNode>* output_graph) const;


	std::map<HexapodMove, std::unique_ptr<INodeCreator>> node_creator_map_;		//!< �m�[�h�����N���X�̃}�b�v�D

	const std::unique_ptr<INodeCreatorBuilder> node_creator_builder_ptr_;			//!< �m�[�h�����N���X�̃r���_�[�D

	const std::shared_ptr<const AbstractHexapodStateCalculator> calculator_ptr_;	//!< �w�L�T�|�b�h�̏�Ԃ��v�Z����N���X�D
};


#endif	//DESIGNLAB_GRAPH_TREE_CREATOR_HATO_H_