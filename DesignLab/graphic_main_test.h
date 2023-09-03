#pragma once

#include <memory>

#include "abstract_graphic_main.h"
#include "hexapod_renderer.h"
#include "node.h"
#include "map_state.h"
#include "camera_gui.h"
#include "node_display_gui.h"
#include "abstract_hexapod_state_calculator.h"


//! @class GraphicMainTest
//! @date 2023/08/09
//! @author ���J��
//! @brief MapState��HexapodStateClaculator�����삵�Ă��邩���o�I�ɕ�����₷�����邽�߂̃e�X�g�V�[��
class GraphicMainTest final : public AbstractGraphicMain
{
public:
	GraphicMainTest(const GraphicDataBroker* const  broker, std::shared_ptr<AbstractHexapodStateCalculator> calc, const SApplicationSettingRecorder* const setting);
	~GraphicMainTest() = default;

	bool update() override;

	void draw() const override;

private:

	HexapodRenderer m_hexapod_renderer;

	MapState m_map_state;

	CameraGUI m_camera_gui;							// �J�����̈ʒu�𐧌䂷��GUI

	NodeDisplayGUI m_node_display_gui;				// �m�[�h�̕\���𐧌䂷��GUI


	SNode m_node;

	int m_map_index = 0;
};


//! @file graphic_main_test.h
//! @date 2023/08/09
//! @author ���J��
//! @brief GraphicMainTest�N���X
//! @n �s�� : @lineinfo
