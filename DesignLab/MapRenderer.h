#pragma once
#include "map_state.h"
#include "Node.h"

// Map�̕`����s���N���X�D
class MapRenderer
{
public:
	MapRenderer();
	~MapRenderer() = default;

	void setNode(const SNode& _node);

	void draw(const MapState& _map) const;

private:

	SNode m_node;
	const unsigned int COLOR_GRAY;
	const unsigned int COLOR_LIGHT_GRAY;
	const float CUBE_SIZE = 15.0f;
};