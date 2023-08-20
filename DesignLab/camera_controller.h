#pragma once

#include "camera_manager.h"


//! @class CameraController
//! @date 2023/08/20
//! @author ���J��
//! @brief �L�[���͂ŃJ�����𓮂����N���X
class CameraController
{
public:
	CameraController() = delete;

	//! @brief �R���X�g���N�^�Ń}�l�[�W���[�̃|�C���^���󂯎��
	CameraController(CameraManager* p_camera_manager);

	//! @brief �L�[���͂ŃJ�����𓮂����D�J�����}�l�[�W���[�̃|�C���^���Ȃ���Α��I������
	//! @n ��{�I�ɂ͖��t���[���Ăяo��
	void update();

private:

	const float kCameraZoomSpeed = 50.0f;	//!< �J�����̃Y�[�����x

	const float kCameraMoveSpeed = 0.007f;	//!< �J�����̈ړ����x


	CameraManager* mp_camera_manager;
};


//! @file camera_controller.h
//! @date 2023/08/20
//! @auther ���J��
//! @brief �J�����𓮂����N���X
