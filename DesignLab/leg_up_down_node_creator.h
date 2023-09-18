#pragma once

#include "interface_node_creator.h"
#include "hexapod_state_calculator.h"


//! @class LegUpDownNodeCreator
//! @date 2023/08/12
//! @author ���J��
//! @brief �r�̏グ����������G�b�W(�ӁC�m�[�h�ƃm�[�h���q����)�̏���������N���X�D
//! @note ��]���l�����Ă��Ȃ��̂Œ���
class LegUpDownNodeCreator final : public INodeCreator
{
public:

	LegUpDownNodeCreator(const MapState* const p_map, const std::shared_ptr<const AbstractHexapodStateCalculator>& calc, const EHexapodMove next_move);
	~LegUpDownNodeCreator() = default;

	void create(const SNode& current_node, int current_node_index, std::vector<SNode>* output_graph) override;

private:

	//�r���ڒn�\�����ׂ�D�n�ʂɊ����邩�ǂ����𒲂ׂĂ��Ȃ��̂Œ��ӁD���ۂɐڒn����Ƃ�����ǂ��ɂȂ邩��output_ground_pos�ŏo�͂���D
	bool isGroundableLeg(int leg_num, const SNode& current_node, dl_vec::SVector* output_ground_pos);

	//���U�������r�ʒu��4�̃O���[�o�����W�C���_�̃O���[�o�����W�C�t�����̃O���[�o�����W�D���݂̋r���(1�`7)�C�����𗘗p���Č��_�����U�������r�ʒu�ɓK���Ă��邩���ׂ�D
	bool isAbleLegPos(const SNode& node, int leg_num);


	const float LEG_MARGIN = 20.0f;		//���ꂾ���������Ό��݂̋r�ʒu�ł��͂��̂Ȃ�΁C�r�ʒu4����ƂȂ�D
	const float HIGH_MARGIN = 5.0f;		//�c�����iZ�������j�̃}�[�W���D���͈͓̔��Ȃ�ΐ^�񒆂ɂ���Ƃ݂Ȃ��D


	const MapState* const mp_map;

	HexapodStateCalclator_Old m_calclator;	//���{�b�g�̍��W�v�Z�N���X�D
	const std::shared_ptr<const AbstractHexapodStateCalculator> mp_calclator;	//���{�b�g�̍��W�v�Z�N���X�D
};


//! @file leg_up_down_node_creator.h
//! @date 2023/08/12
//! @author ���J��
//! @brief �r�̏グ����������G�b�W(�ӁC�m�[�h�ƃm�[�h���q����)�̏���������N���X�D
//! @n �s�� : @lineinfo
