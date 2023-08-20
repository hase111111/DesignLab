#pragma once
#include "interface_node_creator.h"
#include "designlab_polygon.h"
#include "com_type.h"


//! @class ComMoveNodeCreator
//! @date 2023/08/12
//! @author ���J��
//! @brief �d�S�̕��s�ړ����s���N���X�D
class ComMoveNodeCreator final : public INodeCreator
{
public:

	ComMoveNodeCreator(const MapState* const p_map, const EHexapodMove next_move);
	~ComMoveNodeCreator();

	void create(const SNode& current_node, const int current_num, std::vector<SNode>* output_graph) override;

private:

	const MapState* const mp_map;

	std::vector<dl_vec::SPolygon2> m_polygons;

	const bool DO_DEBUG_PRINT = false;
};

//! @file com_move_node_creator.h
//! @date 2023/08/12
//!	@author ���J��
//! @brief �d�S�̕��s�ړ����s���N���X�D
//! @n �s�� : @lineinfo
