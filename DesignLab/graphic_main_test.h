#pragma once

#include "interface_graphic_main.h"
#include "camera_controller.h"
#include "hexapod_renderer.h"
#include "gui_controller.h"
#include "node.h"
#include "map_state.h"


//! @class GraphicMainTest
//! @date 2023/08/09
//! @author ���J��
//! @brief MapState��HexapodStateClaculator�����삵�Ă��邩���o�I�ɕ�����₷�����邽�߂̃e�X�g�V�[��
class GraphicMainTest final : public IGraphicMain
{
public:
	GraphicMainTest(const GraphicDataBroker* broker);
	~GraphicMainTest() = default;

	bool update() override;

	void draw() const override;

private:
	SNode m_node;

	CameraController m_camera_controller;	//�J��������N���X�D
	HexapodRenderer m_hexapod_renderer;
	MapState m_map_state;
	GUIController m_gui_controller;		// GUI (���{�b�g�̏�ԂƂ��\�����鑋) �𐧌䂷��N���X�D

	int m_camera_mode = 0;		//�J�����̃��[�h�D

	int m_map_index = 0;
};


//! @file graphic_main_test.h
//! @date 2023/08/09
//! @author ���J��
//! @brief GraphicMainTest�N���X
//! @n �s�� : @lineinfo
