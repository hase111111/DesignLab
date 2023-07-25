#pragma once
#include "InterfaceNodeCreator.h"
#include "HexapodStateCalculator.h"

class LegUpNodeCreator final : public INodeCreator
{
public:
	LegUpNodeCreator(const MapState* const _p_map, const EHexapodMove _next_move) : INodeCreator(_p_map, _next_move) {};
	~LegUpNodeCreator() = default;

	void create(const SNode& _current_node, const int _current_num, std::vector<SNode>& _output_graph) override;

private:

	HexapodStateCalclator m_calculator;
};