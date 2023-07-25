#pragma once
#include "InterfaceNodeCreator.h"
#include "MyPolygon.h"
#include "ComType.h"
#include "HexapodStateCalculator.h"

class ComMoveNodeCreatorHato final : public INodeCreator
{
public:

	ComMoveNodeCreatorHato(const MapState* const _p_map, const EHexapodMove _next_move) : INodeCreator(_p_map, _next_move), mp_Map(_p_map) {};
	~ComMoveNodeCreatorHato() = default;

	void create(const SNode& _current_node, const int _current_num, std::vector<SNode>& _output_graph) override;

private:

	const MapState* const mp_Map;

	std::vector<my_vec::SPolygon2> m_Polygons;

	const HexapodStateCalclator m_Calclator;

	const bool DO_DEBUG_PRINT = false;

	const float STABLE_MARGIN = 10.0f;	//!< �ÓI���S�]�T 15mm���x���Ó��炵��(�g������̃v���O�������CMAX��40mm���x)

	bool isStable(const SNode _node) const;

	bool isIntersectGround(const SNode _node) const;
};

//! @file ComMoveNodeCreatorHato.h
//! @brief �d�S�̕��s�ړ����s���N���X�D�g������̎�@�D
//! @date 2023/7/25
//!	@auther ���J��

//! @class ComMoveNodeCreatorHato
//! @brief �d�S�̕��s�ړ����s���N���X�D�g������̎�@�D
//! @date 2023/7/25
//! @auther ���J��