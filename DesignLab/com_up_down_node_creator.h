//! @file com_up_down_node_creator.h
//! @brief �d�S�̏グ����������G�b�W(�ӁC�m�[�h�ƃm�[�h���q����)�̏���������N���X�D

#ifndef DESIGNLAB_COM_UP_DOWN_NODE_CREATOR_H_
#define DESIGNLAB_COM_UP_DOWN_NODE_CREATOR_H_


#include "interface_node_creator.h"

#include <memory>

#include "abstract_hexapod_state_calculator.h"
#include "map_state.h"


//! @class ComUpDownNodeCreator
//! @brief �d�S�̏グ����������G�b�W(�ӁC�m�[�h�ƃm�[�h���q����)�̏���������N���X�D
class ComUpDownNodeCreator final : public INodeCreator
{
public:
	ComUpDownNodeCreator(const DevideMapState& devide_map, const std::shared_ptr<const AbstractHexapodStateCalculator>& calc, HexapodMove next_move);
	~ComUpDownNodeCreator() = default;

	void Create(const RobotStateNode& current_node, int current_num, std::vector<RobotStateNode>* output_graph) const override;

private:

	// �O���[�o�����W�̏d�S�̍Œ�ʒu�ƍō��ʒu����C�d�S���㉺�ɕω��������m�[�h��ǉ�����D
	void pushNodeByMaxAndMinPosZ(const RobotStateNode& current_node, int current_num, float high, float low, std::vector<RobotStateNode>* output_graph) const;


	static constexpr int DISCRETIZATION = 5;	//���U�����D�ő�ʒu���ŏ��ʒu������������̂��D

	static constexpr float MARGIN = 10.0f;		//�r��L�΂��؂�Ȃ��悤�ɂ��邽�߂̃}�[�W��[mm]�D���l�͐�y�̃v���O��������Ƃ��Ă����̂łȂ����̐��l���ǂ��̂��͂킩��Ȃ��D


	const DevideMapState map_;

	const std::shared_ptr<const AbstractHexapodStateCalculator> calclator_;	//!< ���{�b�g�̍��W�v�Z�N���X�D

	const HexapodMove next_move_;	//!< ���̓���D
};


#endif	//DESIGNLAB_COM_UP_DOWN_NODE_CREATOR_H_