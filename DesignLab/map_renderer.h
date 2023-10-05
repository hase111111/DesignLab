//! @file map_renderer.h
//! @brief �}�b�v�̕`����s���N���X�D

#ifndef DESIGNLAB_MAP_RENDERER_H_
#define DESIGNLAB_MAP_RENDERER_H_

#include "map_state.h"


//! @class MapRenderer
//! @brief Map�̕`����s���N���X�D
class MapRenderer final
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
	const float CUBE_SIZE;
};


#endif //DESIGNLAB_MAP_RENDERER_H_