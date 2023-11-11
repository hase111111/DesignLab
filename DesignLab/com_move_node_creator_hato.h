//! @file com_move_node_creator_hato.h
//! @brief �d�S�̕��s�ړ����s���N���X�D�g������̎�@�D

#ifndef DESIGNLAB_COM_MOVE_NODE_CREATOR_HATO_H_
#define DESIGNLAB_COM_MOVE_NODE_CREATOR_HATO_H_


#include "interface_node_creator.h"

#include <memory>

#include "com_candidate_polygon_maker.h"
#include "com_selecter_hato.h"
#include "designlab_polygon2.h"
#include "devide_map_state.h"
#include "interface_hexapod_coordinate_converter.h"
#include "interface_hexapod_state_presenter.h"
#include "interface_hexapod_vaild_checker.h"


//! @class ComMoveNodeCreatorHato
//! @brief �d�S�̕��s�ړ����s���N���X�D�g������̎�@�D
class ComMoveNodeCreatorHato final : public INodeCreator
{
public:

	ComMoveNodeCreatorHato(
		const DevideMapState& devide_map, 
		const std::shared_ptr<const IHexapodCoordinateConverter>& converter_ptr,
		const std::shared_ptr<const IHexapodStatePresenter>& presenter_ptr,
		const std::shared_ptr<const IHexapodVaildChecker>& checker_ptr,
		HexapodMove next_move
	);
	~ComMoveNodeCreatorHato() = default;

	void Create(const RobotStateNode& current_node, int current_num, std::vector<RobotStateNode>* output_graph) const override;


private:

	const float kStableMargin;	//!< �ÓI���S�]�T 15mm���x���Ó��炵��(�g������̃v���O�������CMAX��40mm���x)

	const ComCandidatePolygonMaker maker_;	//!< ���n�_���܂ޑ��p�`���쐬����N���X
	const ComSelecterHato selecter_;		//!< ���p�`����œK�Ȓn�ʂ�I������N���X

	const DevideMapState map_;		//!< �n�ʂ̏�Ԃ��i�[�����N���X
	const HexapodMove next_move_;	//!< ���̈ړ�����

	const std::shared_ptr<const IHexapodCoordinateConverter> converter_ptr_;
	const std::shared_ptr<const IHexapodStatePresenter> presenter_ptr_;
	const std::shared_ptr<const IHexapodVaildChecker> checker_ptr_;
};


#endif //DESIGNLAB_COM_MOVE_NODE_CREATOR_HATO_H_