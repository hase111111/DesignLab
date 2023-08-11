#pragma once

#include "map_state.h"
#include "Node.h"


//! @class MapRenderer
//! @date 2023/08/09
//! @auther ���J��
//! @brief Map�̕`����s���N���X�D
class MapRenderer
{
public:
	MapRenderer();
	~MapRenderer() = default;

	void setNode(const SNode& node);

	//! @brief �}�b�v�̕`����s���D
	//! @param [in] map �}�b�v�̏�ԁD
	void draw(const MapState& map) const;

private:

	SNode m_node;
	const unsigned int COLOR_GRAY;
	const unsigned int COLOR_LIGHT_GRAY;
	const float CUBE_SIZE = 15.0f;
};


//! @file map_renderer.h
//! @date 2023/08/09
//! @author	���J��
//! @brief	�}�b�v�̕`����s��MapRenderer�N���X�D
//! @n �s�� : @lineinfo