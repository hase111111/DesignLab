#pragma once

#include <map>
#include <memory>

#include "camera_controller.h"
#include "camera_manager.h"
#include "button_controller.h"
#include "designlab_vector.h"


//! @class CameraGUI
//! @date 2023/08/21
//! @author ���J��
//! @brief �J�����̑���E�Ǘ����s��GUI���s���N���X
class CameraGUI
{
public:

	CameraGUI();

	//! @brief �J�����̒�������ʒu��ݒ肷��
	void setHexapodPos(const dl_vec::SVector pos);

	//! @brief GUI��J�����̍X�V���s��
	void update();

	//! @brief GUI�̕`����s��
	void draw() const;

private:

	//! @brief GUI�̔w�i��`�悷��
	void drawBackground() const;

	//! @brief GUI�̕�����`�悷��
	void drawString() const;


	enum class EButtonType : int
	{
		LENGH_RESET,
		FRONT,
		BACK,
		LEFT,
		RIGHT,
		TOP
	};


	CameraController m_camera_controller;

	CameraManager m_camera_manager;

	std::map<EButtonType, std::unique_ptr<ButtomController>> m_buttons;


	const int kButtonDistance = 10; // �{�^�����m�̊Ԋu

	const int kButtonSize = 50;		// �{�^���̃T�C�Y

	const int kGUILeftPosX = 10;		// GUI�̍��[�̈ʒu

	const int kGUITopPosY = 10;		// GUI�̏�[�̈ʒu
};


//! @file camera_gui.h
//! @date 2023/08/21
//! @auther ���J��
//! @brief �J�����̑���E�Ǘ����s��GUI���s���N���X
//! @n �s�� : @lineinfo
