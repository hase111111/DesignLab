//! @file com_move_node_creator_hato.h
//! @brief �d�S�̕��s�ړ����s���N���X�D�g������̎�@�D

#ifndef DESIGNLAB_COM_MOVE_NODE_CREATOR_HATO_H_
#define DESIGNLAB_COM_MOVE_NODE_CREATOR_HATO_H_


#include "interface_node_creator.h"

#include <memory>

#include "com_type.h"
#include "com_candidate_polygon_maker.h"
#include "com_selecter_hato.h"
#include "designlab_polygon2.h"
#include "hexapod_state_calculator.h"
#include "map_state.h"


//! @class ComMoveNodeCreatorHato
//! @brief �d�S�̕��s�ړ����s���N���X�D�g������̎�@�D
class ComMoveNodeCreatorHato final : public INodeCreator
{
public:

	ComMoveNodeCreatorHato(const DevideMapState& devide_map, const std::shared_ptr<const AbstractHexapodStateCalculator>& calc, EHexapodMove next_move);
	~ComMoveNodeCreatorHato() = default;

	void Create(const SNode& current_node, int current_num, std::vector<SNode>* output_graph) override;


private:

	bool isStable(const SNode& node) const;

	bool isIntersectGround(const SNode& node) const;


	const float kStableMargin;	//!< �ÓI���S�]�T 15mm���x���Ó��炵��(�g������̃v���O�������CMAX��40mm���x)

	//std::vector<designlab::Polygon2> polygon_vec_;


	const DevideMapState map_;	//!< �n�ʂ̏�Ԃ��i�[�����N���X

	const std::shared_ptr<const AbstractHexapodStateCalculator> calculator_ptr_;	//!< ���{�b�g�̏�Ԃ��v�Z����N���X

	const ComCandidatePolygonMaker maker_;	//!< ���n�_���܂ޑ��p�`���쐬����N���X

	ComSelecterHato selecter_;	//!< ���p�`����œK�Ȓn�ʂ�I������N���X

	const EHexapodMove next_move_;	//!< ���̈ړ�����
};


#endif //DESIGNLAB_COM_MOVE_NODE_CREATOR_HATO_H_