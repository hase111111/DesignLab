#pragma once
#include "InterfaceNodeCreator.h"
#include "MyPolygon.h"
#include "ComType.h"


class ComMoveNodeCreator final : public INodeCreator
{
public:

	ComMoveNodeCreator(const MapState* const _p_map, const EHexapodMove _next_move) : INodeCreator(_p_map, _next_move), mp_Map(_p_map) {};
	~ComMoveNodeCreator() = default;

	void create(const SNode& _current_node, const int _current_num, std::vector<SNode>& _output_graph) override;

private:

	const MapState* const mp_Map;

	std::vector<my_vec::SPolygon2> m_Polygons;

	EHexapodMove m_next_move = EHexapodMove::LEG_HIERARCHY_CHANGE;

	const bool DO_DEBUG_PRINT = false;
};

//! @file ComMoveNodeCreator.h
//! @brief 脚の階層構造を作るためのクラス．
//! @date 2023/7/24
//!	@auther 長谷川

//! @class ComMoveNodeCreator
//! @brief 脚の階層構造を作るためのクラス．
//! @date 2023/7/24
//! @auther 長谷川