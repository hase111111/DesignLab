#pragma once
#include "InterfaceNodeCreator.h"
#include "my_polygon.h"
#include "com_type.h"
#include "ComCandidatePolygonMaker.h"
#include "ComSelecterHato.h"
#include "HexapodStateCalculator.h"


class ComMoveNodeCreatorHato final : public INodeCreator
{
public:

	ComMoveNodeCreatorHato(const MapState* const _p_map, const EHexapodMove _next_move);
	~ComMoveNodeCreatorHato();

	void create(const SNode& current_node, const int current_num, std::vector<SNode>* output_graph) override;

private:

	const MapState* const mp_map;

	std::vector<my_vec::SPolygon2> m_Polygons;

	const HexapodStateCalclator m_calclator;

	const ComCandidatePolygonMaker m_maker;

	ComSelecterHato m_selecter;

	static constexpr bool DO_DEBUG_PRINT = false;

	static constexpr float STABLE_MARGIN = 10.0f;	//!< �ÓI���S�]�T 15mm���x���Ó��炵��(�g������̃v���O�������CMAX��40mm���x)

	bool isStable(const SNode& _node) const;

	bool isIntersectGround(const SNode& _node) const;
};

//! @file ComMoveNodeCreatorHato.h
//! @brief �d�S�̕��s�ړ����s���N���X�D�g������̎�@�D
//! @date 2023/7/25
//!	@auther ���J��

//! @class ComMoveNodeCreatorHato
//! @brief �d�S�̕��s�ړ����s���N���X�D�g������̎�@�D
//! @date 2023/7/25
//! @auther ���J��