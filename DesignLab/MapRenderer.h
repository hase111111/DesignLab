#pragma once
#include "MapState.h"

// Map�̕`����s���N���X�D
class MapRenderer
{
public:
	MapRenderer();
	~MapRenderer() = default;

	void draw(const MapState& _map) const;

private:
	const unsigned int COLOR_GRAY;
	const unsigned int COLOR_LIGHT_GRAY;
	const float CUBE_SIZE = 15.0f;
};