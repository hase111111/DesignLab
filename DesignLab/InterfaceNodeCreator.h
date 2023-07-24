#pragma once
#include "Node.h"
#include "MapState.h"
#include "HexapodNextMove.h"

class INodeCreator
{
public:
	INodeCreator(const MapState* const _p_Map, const EHexapodMove _next_move) : m_next_move(_next_move) {};
	virtual ~INodeCreator() = default;

	//! @brief �d�S�𕽍s�ړ������m�[�h�𐶐�����
	//! @param[in] _current_node �d�S�𕽍s�ړ�����m�[�h
	//! @param[in] _current_num �d�S�𕽍s�ړ�����m�[�h�̔ԍ�
	//! @param[out] _output_graph �d�S�𕽍s�ړ������m�[�h���i�[����R���e�i
	virtual void create(const SNode& _current_node, const int _current_num, std::vector<SNode>& _output_graph) = 0;

protected:
	const EHexapodMove m_next_move;

};
