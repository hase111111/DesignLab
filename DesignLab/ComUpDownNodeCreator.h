#pragma once
#include "MapState.h"

class ComUpDownNodeCreator final
{
public:
	void init();

	void create(const SNode& _current_node, const int _current_num, std::vector<SNode>& _output_graph);

private:

	const MapState* mp_Map;

	const int DISCRETIZATION = 5;	//���U�����D�ő�ʒu���ŏ��ʒu������������̂��D
};
