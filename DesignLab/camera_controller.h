//! @file camera_controller.h
//! @brief �}�E�X�̓��͂ŃJ�����𓮂����N���X

#ifndef DESIGNLABO_CAMERA_CONTROLLER_H_
#define DESIGNLABO_CAMERA_CONTROLLER_H_

#include "camera_state_manager.h"


//! @class CameraController
//! @brief �}�E�X�̓��͂ŃJ�����𓮂����N���X

class CameraController
{
public:
	//! @brief �f�t�H���g�R���X�g���N�^�C�R�s�[�R���X�g���N�^�C������Z�q�͋֎~
	CameraController() = delete;
	CameraController(const CameraController&) = delete;
	CameraController& operator=(const CameraController&) = delete;

	//! @brief �R���X�g���N�^�Ń}�l�[�W���[�̎Q�Ƃ��󂯎��
	CameraController(CameraStateManager& camera_manager_ref);


	//! @brief �L�[���͂ŃJ�����𓮂����D
	//! @n ��{�I�ɂ͖��t���[���Ăяo��
	void Update();

private:

	const float kCameraZoomSpeed = 50.0f;		//!< �J�����̃Y�[�����x

	const float kCameraMoveSpeed = 0.007f;		//!< �J�����̈ړ����x

	const float kCameraTargetMoveSpeed = 3.0f;	//!< �J�����̒����_�̈ړ����x

	const double kMouseMoveMargin = 2.0;		//!< �}�E�X�̈ړ��ʂ����̗ʈȉ��Ȃ��0�Ƃ݂Ȃ�


	CameraStateManager& camera_manager_ref_;
};


#endif // !DESIGNLABO_CAMERA_CONTROLLER_H_