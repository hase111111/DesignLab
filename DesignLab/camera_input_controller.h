//! @file camera_input_controller.h
//! @brief �}�E�X�̓��͂ŃJ�����𓮂����N���X

#ifndef DESIGNLAB_CAMERA_INPUT_CONTROLLER_H_
#define DESIGNLAB_CAMERA_INPUT_CONTROLLER_H_


#include "camera_state_manager.h"


//! @class CameraInputController
//! @brief �}�E�X�̓��͂ŃJ�����𓮂����N���X
class CameraInputController
{
public:

	CameraInputController();

	//! @brief �L�[���͂ŃJ�����𓮂����D
	//! @n ��{�I�ɂ͖��t���[���Ăяo��
	//! @param [out] camera_manager �J�����̏�Ԃ��Ǘ�����N���X�̃|�C���^�D�󂯎�����l������������
	void ChangeCameraState(CameraStateManager* camera_manager);

private:

	const float kCameraZoomSpeed;		//!< �J�����̃Y�[�����x

	const float kCameraMoveSpeed;		//!< �J�����̈ړ����x

	const float kCameraTargetMoveSpeed;	//!< �J�����̒����_�̈ړ����x

	const double kMouseMoveMargin;		//!< �}�E�X�̈ړ��ʂ����̗ʈȉ��Ȃ��0�Ƃ݂Ȃ�
};


#endif	// DESIGNLAB_CAMERA_INPUT_CONTROLLER_H_