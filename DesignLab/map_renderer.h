#pragma once

#include "map_state.h"
#include "Node.h"


//! @class MapRenderer
//! @date 2023/08/09
//! @author ���J��
//! @brief Map�̕`����s���N���X�D
class MapRenderer
{
public:
	MapRenderer();
	~MapRenderer() = default;

	//! @brief �}�b�v�̕`����s���D
	//! @param [in] map �}�b�v�̏�ԁD
	void Draw(const MapState& map) const;

private:

	const unsigned int COLOR_GRAY;
	const unsigned int COLOR_LIGHT_GRAY;
	const float CUBE_SIZE = 15.0f;
};


//! @file map_renderer.h
//! @date 2023/08/09
//! @author	���J��
//! @brief	�}�b�v�̕`����s��MapRenderer�N���X�D
//! @n �s�� : @lineinfo