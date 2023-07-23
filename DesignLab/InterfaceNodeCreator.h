#pragma once
#include "Node.h"
#include "MapState.h"

class INodeCreator
{
public:
	INodeCreator(const MapState* const _p_Map) {};
	virtual ~INodeCreator() = default;

	//! @brief �d�S�𕽍s�ړ������m�[�h�𐶐�����
	//! @param[in] _current_node �d�S�𕽍s�ړ�����m�[�h
	//! @param[in] _current_num �d�S�𕽍s�ړ�����m�[�h�̔ԍ�
	//! @param[out] _output_graph �d�S�𕽍s�ړ������m�[�h���i�[����R���e�i
	virtual void create(const SNode& _current_node, const int _current_num, std::vector<SNode>& _output_graph) = 0;


};
