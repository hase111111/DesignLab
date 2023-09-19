#pragma once

#include <vector>
#include <memory>

#include "node.h"
#include "map_state.h"
#include "hexapod_next_move.h"
#include "abstract_hexapod_state_calculator.h"


//! @class INodeCreator
//! @date 2023/08/12
//! @author ���J��
//! @brief �m�[�h�����C���^�[�t�F�[�X
class INodeCreator
{
public:

	//! @brief �R���X�g���N�^�ł͎������ݒ肷��D�܂��}�b�v�̃|�C���^���󂯎��
	INodeCreator(const MapState_Old* const p_map, const std::shared_ptr<const AbstractHexapodStateCalculator>& calc, const EHexapodMove next_move) : m_next_move(next_move) {};
	virtual ~INodeCreator() = default;


	//! @brief �d�S�𕽍s�ړ������m�[�h�𐶐�����
	//! @param[in] current_node �d�S�𕽍s�ړ�����m�[�h
	//! @param[in] current_node_index �d�S�𕽍s�ړ�����m�[�h�̔ԍ�
	//! @param[out] output_graph �d�S�𕽍s�ړ������m�[�h���i�[����R���e�i
	virtual void create(const SNode& current_node, const int current_node_index, std::vector<SNode>* output_graph) = 0;

protected:

	const EHexapodMove m_next_move;	//!< ������
};


//! @file interface_node_creator.h
//! @date 2023/08/12
//! @author ���J��
//! @brief �m�[�h�����C���^�[�t�F�[�X
//! @n �s�� : @lineinfo
