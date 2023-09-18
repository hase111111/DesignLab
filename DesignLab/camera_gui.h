#pragma once

#include <map>
#include <memory>

#include "camera_controller.h"
#include "camera_manager.h"
#include "button_controller.h"
#include "designlab_vector.h"


//! @class CameraGui
//! @date 2023/08/21
//! @author ���J��
//! @brief �J�����̑���E�Ǘ����s��GUI���s���N���X
class CameraGui final
{
public:

	CameraGui(const int left_x, const int top_y);
	CameraGui();

	//! @brief �J�����̒�������ʒu��ݒ肷��
	void setHexapodPos(const dl_vec::SVector pos);

	//! @brief GUI��J�����̍X�V���s��
	void Update();

	//! @brief GUI�̕`����s��
	void Draw() const;

	const static int GUI_SIZE_X = 235;
	const static int GUI_SIZE_Y = 410;
	const static int CLOSED_GUI_SIZE_Y = 40;

private:

	//! @brief GUI�̔w�i��`�悷��
	void DrawBackground() const;

	//! @brief �ŏ�������GUI�̔w�i��`�悷��
	void drawClosedBackground() const;

	//! @brief GUI�̕�����`�悷��
	void drawString() const;


	enum class EButtonType : int
	{
		LENGH_RESET,
		FRONT,
		BACK,
		LEFT,
		RIGHT,
		TOP,
		TARGET_RESET,
		CLOSED
	};


	CameraController m_camera_controller;	//!< �J�����̑�����s���N���X

	CameraStateManager m_camera_manager;			//!< �J�����̊Ǘ����s���N���X

	std::map<EButtonType, std::unique_ptr<ButtomController>> m_buttons;


	const int kButtonDistance = 10; // �{�^�����m�̊Ԋu

	const int kButtonSize = 60;		// �{�^���̃T�C�Y

	const int kGuiLeftPosX;			// GUI�̍��[�̈ʒu

	const int kGuiTopPosY;			// GUI�̏�[�̈ʒu


	bool m_is_closed = false;
};


//! @file camera_gui.h
//! @date 2023/08/21
//! @auther ���J��
//! @brief �J�����̑���E�Ǘ����s��GUI���s���N���X
//! @n �s�� : @lineinfo
