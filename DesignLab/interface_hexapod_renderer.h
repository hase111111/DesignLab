//! @file interface_hexapod_renderer.h
//! @brief 6�r���{�b�g�̕`��̃C���^�[�t�F�[�X


#ifndef INTERFACE_HEXAPOD_RENDERER_H_
#define INTERFACE_HEXAPOD_RENDERER_H_

#include "robot_state_node.h"


//! @class IHexapodRenderer
//! @brief 6�r���{�b�g�̕`��̃C���^�[�t�F�[�X
class IHexapodRenderer
{
public:

	virtual ~IHexapodRenderer() = default;

	//! @brief ���{�b�g�̏�Ԃ��X�V����D
	//! @n �X�V�̍ۂɁC�W���C���g�̌v�Z������\�������邽�߁C�v�Z���ׂ������D�A�����ČĂяo���Ȃ����ƁD
	//! @param [in] node �`�悷�郍�{�b�g�̏��
	virtual void SetDrawNode(const RobotStateNode& node) = 0;

	//! @brief ���{�b�g��3D��Ԃɕ`�悷��D
	virtual void Draw() const = 0;
};


#endif	// #ifndef INTERFACE_HEXAPOD_RENDERER_H_