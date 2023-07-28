#pragma once
#include "Node.h"
#include "MapState.h"
#include "HexapodNextMove.h"

class INodeCreator
{
public:
	INodeCreator(const MapState* const p_map, const EHexapodMove next_move) : m_next_move(next_move) {};
	virtual ~INodeCreator() = default;

	//! @brief �d�S�𕽍s�ړ������m�[�h�𐶐�����
	//! @param[in] current_node �d�S�𕽍s�ړ�����m�[�h
	//! @param[in] current_node_index �d�S�𕽍s�ړ�����m�[�h�̔ԍ�
	//! @param[out] output_graph �d�S�𕽍s�ړ������m�[�h���i�[����R���e�i
	virtual void create(const SNode& current_node, const int current_node_index, std::vector<SNode>* output_graph) = 0;

protected:
	const EHexapodMove m_next_move;

};
