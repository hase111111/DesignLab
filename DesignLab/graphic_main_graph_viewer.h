#pragma once

#include <memory>

#include "interface_graphic_main.h"
#include "HexapodRenderer.h"
#include "map_state.h"
#include "CameraController.h"
#include "GraphViewerGUIController.h"


//! @class GraphicMainGraphViewer
//! @date 2023/08/09
//! @author ���J��
//! @brief GraphViewer�̉�ʂ�\������N���X
class GraphicMainGraphViewer final : public IGraphicMain
{
public:
	GraphicMainGraphViewer(const GraphicDataBroker* broker);
	~GraphicMainGraphViewer() = default;

	bool update() override;

	void draw() const override;

private:
	HexapodRenderer m_hexapod_renderer;
	MapState m_map_state;
	CameraController m_camera_controller;
	std::unique_ptr<GraphViewerGUIController> mp_gui_controller;

	std::vector<SNode> m_graph;
	size_t m_display_node_index = 0;

	void updateCameraState();
	int m_camera_mode = 0;
};


//! @file graphic_main_graph_viewer.h
//! @date 2023/08/09
//! @auther ���J��
//! @brief GraphicMainGraphViewer�N���X
//! @n �s�� : @lineinfo
