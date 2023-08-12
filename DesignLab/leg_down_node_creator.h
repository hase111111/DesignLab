#pragma once

#include "interface_node_creator.h"
#include "HexapodStateCalculator.h"


//! @class LegUpDownNodeCreator
//! @date 2023/08/12
//! @author ���J��
//! @brief �r�̏グ����������G�b�W(�ӁC�m�[�h�ƃm�[�h���q����)�̏���������N���X�D
class LegDownNodeCreator final : public INodeCreator
{
public:
	LegDownNodeCreator(const MapState* const p_map, const EHexapodMove next_move) : INodeCreator(p_map, next_move), mp_map(p_map) {};
	~LegDownNodeCreator() = default;

	void create(const SNode& current_node, const int current_num, std::vector<SNode>* output_graph) override;

private:

	//�r���ڒn�\�����ׂ�D�n�ʂɊ����邩�ǂ����𒲂ׂĂ��Ȃ��̂Œ��ӁD���ۂɐڒn����Ƃ�����ǂ��ɂȂ邩��output_ground_pos�ŏo�͂���D
	bool isGroundableLeg(const int leg_num, const SNode& current_node, my_vec::SVector& output_ground_pos);

	//���U�������r�ʒu��4�̃O���[�o�����W�C���_�̃O���[�o�����W�C�t�����̃O���[�o�����W�D���݂̋r���(1�`7)�C�����𗘗p���Č��_�����U�������r�ʒu�ɓK���Ă��邩���ׂ�D
	bool isAbleLegPos(const SNode& node, const int leg_num);


	const float LEG_MARGIN = 20.0f;		//���ꂾ���������Ό��݂̋r�ʒu�ł��͂��̂Ȃ�΁C�r�ʒu4����ƂȂ�D

	const float HIGH_MARGIN = 5.0f;		//�c�����iZ�������j�̃}�[�W���D���͈͓̔��Ȃ�ΐ^�񒆂ɂ���Ƃ݂Ȃ��D


	HexapodStateCalclator m_calculator;

	const MapState* const mp_map;

};


//! @file leg_up_down_node_creator.h
//! @date 2023/08/12
//! @author ���J��
//! @brief �r�̏グ����������G�b�W(�ӁC�m�[�h�ƃm�[�h���q����)�̏���������N���X�D
//! @n �s�� : @lineinfo
