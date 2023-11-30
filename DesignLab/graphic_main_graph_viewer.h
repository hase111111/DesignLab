﻿//! @file graphic_main_graph_viewer.h
//! @brief GraphViewerの画面を表示するクラス

#ifndef DESIGNLAB_GRAPHIC_MAIN_GRAPH_VIEWER_H_
#define DESIGNLAB_GRAPHIC_MAIN_GRAPH_VIEWER_H_

#include "interface_graphic_main.h"

#include <memory>

#include "camera_gui.h"
#include "graph_viewer_gui_controller.h"
#include "graphic_data_broker.h"
#include "dxlib_gui_updater.h"
#include "interface_hexapod_coordinate_converter.h"
#include "interface_hexapod_joint_calculator.h"
#include "interface_hexapod_vaild_checker.h"
#include "map_state.h"
#include "mouse.h"
#include "node_display_gui.h"


//! @class GraphicMainGraphViewer
//! @brief GraphViewerの画面を表示するクラス
class GraphicMainGraphViewer final : public IGraphicMain
{
public:
	GraphicMainGraphViewer(
		const std::shared_ptr<const GraphicDataBroker>& broker_ptr,
		const std::shared_ptr<const IHexapodCoordinateConverter>& converter_ptr,
		const std::shared_ptr<const IHexapodJointCalculator>& calculator_ptr,
		const std::shared_ptr<const IHexapodVaildChecker>& checker_ptr,
		const std::shared_ptr<const ApplicationSettingRecorder>& setting_ptr
	);

	~GraphicMainGraphViewer() = default;

	bool Update() override;

	void Draw() const override;

private:

	const std::shared_ptr<const GraphicDataBroker> broker_ptr_;

	std::shared_ptr<Mouse> mouse_ptr_;

	std::shared_ptr<DxlibCamera> camera_;
	std::shared_ptr<CameraGui> camera_gui_;
	std::shared_ptr<NodeDisplayGui> node_display_gui_;
	const std::unique_ptr<GraphViewerGUIController> gui_controller_ptr_;

	DxlibGuiUpdater gui_activator_;


	MapState map_state_;


	std::vector<RobotStateNode> graph_;

	size_t display_node_index_ = 0;


	int map_update_count_;

	int graph_update_count_;
};


#endif // DESIGNLAB_GRAPHIC_MAIN_GRAPH_VIEWER_H_