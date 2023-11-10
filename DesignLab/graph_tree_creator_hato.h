//! @file graph_tree_creator_hato.h 
//! @brief �g������̃O���t���쐬����v���O�������ڐA�����N���X�̎���

#ifndef DESIGNLAB_GRAPH_TREE_CREATOR_HATO_H_
#define DESIGNLAB_GRAPH_TREE_CREATOR_HATO_H_

#include "interface_graph_tree_creator.h"

#include <map>
#include <memory>

#include "interface_hexapod_coordinate_converter.h"
#include "interface_hexapod_state_presenter.h"
#include "interface_hexapod_vaild_checker.h"
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
		const std::shared_ptr<const IHexapodCoordinateConverter>& converter_ptr,
		const std::shared_ptr<const IHexapodStatePresenter>& presenter_ptr,
		const std::shared_ptr<const IHexapodVaildChecker>& checker_ptr
	);

	~GraphTreeCreatorHato() = default;


	void Init(const DevideMapState& map_state);

	GraphSearchResult CreateGraphTree(const RobotStateNode& current_node, int max_depth, std::vector<RobotStateNode>* output_graph) override;

private:

	//! @brief ���݂̃O���t����C�ő�[���܂Ńm�[�h�𐶐�����D
	//! @param[in] max_depth �ő�[��
	//! @param[out] output_graph ���������m�[�h��ǉ�����x�N�^�D
	void MakeGraphTreeToMaxDepth(int max_depth, std::vector<RobotStateNode>* output_graph) const;

	//! @brief current_node�̎q�m�[�h�𐶐����āCoutput_graph�ɑ������D
	//! @param[in] current_node ���݂̃m�[�h
	//! @param[in] current_num ���݂̃m�[�h��index
	//! @param[out] output_graph ���������m�[�h��������x�N�^�D��ɂ��Ă�������.
	void MakeNewNodesByCurrentNode(const RobotStateNode& current_node, int current_index, std::vector<RobotStateNode>* output_graph) const;


	constexpr static int kMultiThreadNum = 6;		//!< �}���`�X���b�h�̐��D

	std::map<HexapodMove, std::unique_ptr<INodeCreator>> node_creator_map_;		//!< �m�[�h�����N���X�̃}�b�v�D

	const std::unique_ptr<INodeCreatorBuilder> node_creator_builder_ptr_;		//!< �m�[�h�����N���X�̃r���_�[�D

	const std::shared_ptr<const IHexapodCoordinateConverter> converter_ptr_;
	const std::shared_ptr<const IHexapodStatePresenter> presenter_ptr_;
	const std::shared_ptr<const IHexapodVaildChecker> checker_ptr_;
};


#endif	//DESIGNLAB_GRAPH_TREE_CREATOR_HATO_H_