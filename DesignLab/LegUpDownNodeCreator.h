#pragma once
#include "InterfaceNodeCreator.h"
#include "HexapodStateCalculator.h"


class LegUpDownNodeCreator final : public INodeCreator
{
private:

	const float LEG_MARGIN = 20.0f;		//���ꂾ���������Ό��݂̋r�ʒu�ł��͂��̂Ȃ�΁C�r�ʒu4����ƂȂ�D
	const float HIGH_MARGIN = 5.0f;		//�c�����iZ�������j�̃}�[�W���D���͈͓̔��Ȃ�ΐ^�񒆂ɂ���Ƃ݂Ȃ��D

public:

	LegUpDownNodeCreator(const MapState* const _p_Map, const EHexapodMove _next_move) : INodeCreator(_p_Map, _next_move), mp_Map(_p_Map) {};
	~LegUpDownNodeCreator() = default;

	void create(const SNode& _current_node, const int _current_num, std::vector<SNode>& _output_graph) override;

private:

	const MapState* const mp_Map;

	HexapodStateCalclator m_Calc;	//���{�b�g�̍��W�v�Z�N���X�D

	constexpr EHexapodMove getNextMove(const EHexapodMove& _last_move) const
	{
		if (_last_move == EHexapodMove::LEG_UP_DOWN_NEXT_COM_MOVE)
		{
			return EHexapodMove::COM_MOVE;
		}
		else
		{
			return EHexapodMove::COM_UP_DOWN;
		}
	}

	//�r���ڒn�\�����ׂ�D�n�ʂɊ����邩�ǂ����𒲂ׂĂ��Ȃ��̂Œ��ӁD���ۂɐڒn����Ƃ�����ǂ��ɂȂ邩��output_ground_pos�ŏo�͂���D
	bool isGroundableLeg(const int _leg_num, const SNode& _current_node, my_vec::SVector& _output_ground_pos);

	//���U�������r�ʒu��4�̃O���[�o�����W�C���_�̃O���[�o�����W�C�t�����̃O���[�o�����W�D���݂̋r���(1�`7)�C�����𗘗p���Č��_�����U�������r�ʒu�ɓK���Ă��邩���ׂ�D
	bool isAbleLegPos(const SNode& _node, const int _leg_num);
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