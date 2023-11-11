//! @file graph_tree_creator_basic.h 
//! @brief ���e�p�^�[���O���t���쐬����N���X

#ifndef DESIGNLAB_GRAPH_TREE_CREATOR_BASIC_H_
#define DESIGNLAB_GRAPH_TREE_CREATOR_BASIC_H_

#include "interface_graph_tree_creator.h"

#include <map>
#include <memory>

#include "interface_node_creator.h"
#include "interface_node_creator_builder.h"


//! @class GraphTreeCreatorBasic
//! @brief ���e�p�^�[���O���t���쐬����N���X
//! @details ��s�����̃v���O����������΂킩��ʂ�C���ۂɂ͏��������������邽�߂ɁC
//! @n �����X���b�h�����ɏ������s���̂����C���̃N���X�ł͒P��̃X���b�h�ŏ������s���D
class GraphTreeCreatorBasic final : public IGraphTreeCreator
{
public:
	GraphTreeCreatorBasic(
		std::unique_ptr<INodeCreatorBuilder>&& node_creator_builder_ptr,
		const std::shared_ptr<const IHexapodCoordinateConverter>& converter_ptr,
		const std::shared_ptr<const IHexapodStatePresenter>& presenter_ptr,
		const std::shared_ptr<const IHexapodVaildChecker>& checker_ptr
	);

	~GraphTreeCreatorBasic() = default;


	void Init(const DevideMapState& map_state);

	GraphSearchResult CreateGraphTree(const RobotStateNode& current_node, int max_depth, std::vector<RobotStateNode>* output_graph) override;

private:

	//! @brief current_node�̎q�m�[�h�𐶐����āCoutput_graph�ɑ������D
	//! @param[in] current_node ���݂̃m�[�h
	//! @param[in] current_num ���݂̃m�[�h��index
	//! @param[out] output_graph ���������m�[�h��������x�N�^�D��ɂ��Ă�������.
	void MakeNewNodesByCurrentNode(const RobotStateNode& current_node, const int current_index, std::vector<RobotStateNode>* output_graph) const;


	std::map<HexapodMove, std::unique_ptr<INodeCreator>> node_creator_map_;		//!< �m�[�h�����N���X�̃}�b�v�D

	const std::unique_ptr<INodeCreatorBuilder> node_creator_builder_ptr_;		//!< �m�[�h�����N���X�̃r���_�[�D

	const std::shared_ptr<const IHexapodCoordinateConverter> converter_ptr_;
	const std::shared_ptr<const IHexapodStatePresenter> presenter_ptr_;
	const std::shared_ptr<const IHexapodVaildChecker> checker_ptr_;
};


#endif	// DESIGNLAB_GRAPH_TREE_CREATOR_BASIC_H_