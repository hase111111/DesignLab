//! @file leg_up_down_node_creator.h
//! @brief �r�̏グ����������G�b�W(�ӁC�m�[�h�ƃm�[�h���q����)�̏���������N���X�D

#ifndef DESIGNLAB_LEG_UP_DOWN_NODE_CREATOR_H_
#define DESIGNLAB_LEG_UP_DOWN_NODE_CREATOR_H_


#include "interface_node_creator.h"

#include <memory>

#include "abstract_hexapod_state_calculator.h"
#include "map_state.h"


//! @class LegUpDownNodeCreator
//! @brief �r�̏グ����������G�b�W(�ӁC�m�[�h�ƃm�[�h���q����)�̏���������N���X�D
//! @note ��]���l�����Ă��Ȃ��̂Œ���
class LegUpDownNodeCreator final : public INodeCreator
{
public:

	LegUpDownNodeCreator(const DevideMapState& devide_map, const std::shared_ptr<const AbstractHexapodStateCalculator>& calc, HexapodMove next_move);
	~LegUpDownNodeCreator() = default;

	void Create(const RobotStateNode& current_node, int current_node_index, std::vector<RobotStateNode>* output_graph) const override;

private:

	//�r���ڒn�\�����ׂ�D�n�ʂɊ����邩�ǂ����𒲂ׂĂ��Ȃ��̂Œ��ӁD���ۂɐڒn����Ƃ�����ǂ��ɂȂ邩��output_ground_pos�ŏo�͂���D
	bool IsGroundableLeg(int leg_num, const RobotStateNode& current_node, designlab::Vector3* output_ground_pos) const;

	//���U�������r�ʒu��4�̃O���[�o�����W�C���_�̃O���[�o�����W�C�t�����̃O���[�o�����W�D���݂̋r���(1�`7)�C�����𗘗p���Č��_�����U�������r�ʒu�ɓK���Ă��邩���ׂ�D
	bool IsAbleLegPos(const RobotStateNode& node, int leg_num) const;


	const float kLegMargin;		//!< ���ꂾ���������Ό��݂̋r�ʒu�ł��͂��̂Ȃ�΁C�r�ʒu4����ƂȂ�D
	const float kHighMargin;	//!< �c�����iZ�������j�̃}�[�W���D���͈͓̔��Ȃ�ΐ^�񒆂ɂ���Ƃ݂Ȃ��D

	const DevideMapState map_;	//!< �}�b�v�̏�ԁD

	const std::shared_ptr<const AbstractHexapodStateCalculator> calclator_ptr_;	//!< ���{�b�g�̍��W�v�Z�N���X�D

	const HexapodMove next_move_;	//!< ���̈ړ������D
};


#endif	//DESIGNLAB_LEG_UP_DOWN_NODE_CREATOR_H_