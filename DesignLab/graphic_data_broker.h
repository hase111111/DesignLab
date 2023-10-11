//! @file graphic_data_broker.h
//! @brief �O���t�T���̌��ʂ�ʃX���b�h�̃O���t�B�b�N�N���X�ɘA�����邽�߂̍\����

#ifndef DESIGNLAB_GRAPHIC_DATA_BROKER_H_
#define DESIGNLAB_GRAPHIC_DATA_BROKER_H_


#include "asyncable_data.h"
#include "map_state.h"
#include "robot_state_node.h"


//! @struct GraphicDataBroker
//! @brief �摜�\�����ƁC�f�[�^�����������т��钇��l
//! @details Broker:�u���[�J�[�C����l�̂��ƁD
//! @n �f�[�^������(�O���t�T��)���X�V�����f�[�^�����̍\���̂ɓn���C�摜�\���������̍\���̂���X�V���ꂽ�f�[�^�������Ă����C�`�悷��D
//! @n
//! @n �O���t�T���@���@GraphicDataBroker�@���@�摜�\����
struct GraphicDataBroker final
{
	AsyncableData<MapState> map_state;					//!< �}�b�v�̏��C�J���������{�b�g�ɐݒu����ꍇ�C

	AsyncableData<std::vector<RobotStateNode>> graph;

	AsyncableData<std::vector<size_t>> simu_end_index;
};


#endif // !DESIGNLAB_GRAPHIC_DATA_BROKER_H_