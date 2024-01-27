﻿
/// @file      graphic_main_robot_control.h
/// @author    hasegawa
/// @copyright © 埼玉大学 設計工学研究室 2024. All right reserved.

#ifndef DESIGNLAB_GRAPHIC_MAIN_ROBOT_CONTROL_H_
#define DESIGNLAB_GRAPHIC_MAIN_ROBOT_CONTROL_H_

#include <memory>

#include "application_setting_record.h"
#include "divided_map_state.h"
#include "dxlib_node_setter_group.h"
#include "dxlib_gui_updater.h"
#include "dxlib_gui_display_node_switcher.h"
#include "dxlib_3d_renderer_group.h"
#include "graphic_data_broker.h"
#include "interface_graphic_main.h"
#include "interface_hexapod_coordinate_converter.h"
#include "interface_hexapod_joint_calculator.h"
#include "interface_hexapod_posture_validator.h"
#include "map_state.h"
#include "mouse.h"


namespace designlab
{

class GraphicMainRobotControl final : public IGraphicMain
{
public:
    GraphicMainRobotControl(const std::shared_ptr<GraphicDataBroker>& broker,
                            const std::shared_ptr<const IHexapodCoordinateConverter>& converter_ptr,
                            const std::shared_ptr<const IHexapodJointCalculator>& calculator_ptr,
                            const std::shared_ptr<const IHexapodPostureValidator>& checker_ptr,
                            const std::shared_ptr<const ApplicationSettingRecord>& setting_ptr);
    ~GraphicMainRobotControl() = default;


    bool Update() override;
    void Draw() const override;

private:
    std::shared_ptr<Mouse> mouse_ptr_;  //!< マウスの位置を制御するクラス．

    DxlibGuiUpdater gui_updater_;  //!< GUIの更新を行うクラス．
    DxlibNodeSetterGroup node_setter_group_;  //!< ノードの設定を行うクラス．
    Dxlib3dRendererGroup render_group_;  //!< 描画を行うクラス．

    const std::shared_ptr<const IHexapodCoordinateConverter> converter_ptr_;
    const std::shared_ptr<const IHexapodJointCalculator> calculator_ptr_;

    std::shared_ptr<DxlibGuiDisplayNodeSwitcher> display_node_switch_gui_;
    MapState map_state_;  //!< マップの状態を保持するクラス．
    DividedMapState divided_map_state_;  //!< 分割したマップの状態を保持するクラス．

    std::vector<RobotStateNode> graph_;  //!< ロボットの動きの遷移を記録する配列．

    const std::shared_ptr<GraphicDataBroker> broker_;

    int graph_update_count_{ 0 };
};

}  // namespace designlab

#endif  // DESIGNLAB_GRAPHIC_MAIN_ROBOT_CONTROL_H_