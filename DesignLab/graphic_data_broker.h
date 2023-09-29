//! @file graphic_data_broker.h
//! @brief �O���t�T���̌��ʂ�ʃX���b�h�̃O���t�B�b�N�N���X�ɘA������N���X�D

#ifndef DESIGNLAB_GRAPHIC_DATA_BROKER_H_
#define DESIGNLAB_GRAPHIC_DATA_BROKER_H_


#include "asyncable_data.h"
#include "map_state.h"
#include "robot_state_node.h"


//! @struct GraphicDataBroker
//! @brief �摜�\�����ƁC�f�[�^�����������т��钇��l�N���X
//! @details Broker:�u���[�J�[�C����l�̂��ƁD
//! @n �f�[�^������(�O���t�T��)���X�V�����f�[�^�����̃N���X�ɓn���C�摜�\���������̃N���X����X�V���ꂽ�f�[�^�������Ă����C�`�悷��D
//! @n
//! @n �O���t�T���@���@GraphicDataBroker�@���@�摜�\����
struct GraphicDataBroker final
{
	AsyncableData<MapState> map_state;

	AsyncableData<std::vector<RobotStateNode>> graph;

	AsyncableData<std::vector<size_t>> simu_end_index;
};

#endif // !DESIGNLAB_GRAPHIC_DATA_BROKER_H_