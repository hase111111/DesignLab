#pragma once
#include "InterfaceNodeCreator.h"
#include "HexapodStateCalculator.h"


class LegUpDownNodeCreator final : public INodeCreator
{
private:

	const float LEG_MARGIN = 20.0f;		//���ꂾ���������Ό��݂̋r�ʒu�ł��͂��̂Ȃ�΁C�r�ʒu4����ƂȂ�D
	const float HIGH_MARGIN = 5.0f;		//�c�����iZ�������j�̃}�[�W���D���͈͓̔��Ȃ�ΐ^�񒆂ɂ���Ƃ݂Ȃ��D

public:

	LegUpDownNodeCreator(const MapState* const p_Map, const EHexapodMove next_move) : INodeCreator(p_Map, next_move), mp_map(p_Map) {};
	~LegUpDownNodeCreator() = default;

	void create(const SNode& current_node, const int current_node_index, std::vector<SNode>* output_graph) override;

private:

	const MapState* const mp_map;

	HexapodStateCalclator m_calclator;	//���{�b�g�̍��W�v�Z�N���X�D

	//�r���ڒn�\�����ׂ�D�n�ʂɊ����邩�ǂ����𒲂ׂĂ��Ȃ��̂Œ��ӁD���ۂɐڒn����Ƃ�����ǂ��ɂȂ邩��output_ground_pos�ŏo�͂���D
	bool isGroundableLeg(const int leg_num, const SNode& current_node, my_vec::SVector* output_ground_pos);

	//���U�������r�ʒu��4�̃O���[�o�����W�C���_�̃O���[�o�����W�C�t�����̃O���[�o�����W�D���݂̋r���(1�`7)�C�����𗘗p���Č��_�����U�������r�ʒu�ɓK���Ă��邩���ׂ�D
	bool isAbleLegPos(const SNode& node, const int leg_num);
};


//! @file LegUpDownNodeCreator.h
//! @brief �r�̏グ����������G�b�W(�ӁC�m�[�h�ƃm�[�h���q����)�̏���������N���X�D
//! @date 2023/7/24
//! @auther ���J��

//! @class LegUpDownNodeCreator
//! @brief �r�̏グ����������G�b�W(�ӁC�m�[�h�ƃm�[�h���q����)�̏���������N���X�D
//! @note ��]���l�����Ă��Ȃ��̂Œ���
//! @date 2023/7/24
//! @auther ���J��