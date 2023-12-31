﻿
//! @file      interpolated_node_creator.h
//! @author    Hasegawa
//! @copyright © 埼玉大学 設計工学研究室 2023. All right reserved.

#ifndef DESIGNLAB_INTERPOLATED_NODE_CREATOR_H_
#define DESIGNLAB_INTERPOLATED_NODE_CREATOR_H_

#include <array>
#include <memory>
#include <vector>

#include "interface_hexapod_coordinate_converter.h"
#include "robot_state_node.h"


namespace designlab
{

//! @class InterpolatedNodeCreator
//! @brief 矩形軌道を生成し，ノード間を補間するクラス．
class InterpolatedNodeCreator final
{
public:
    explicit InterpolatedNodeCreator(
        const std::shared_ptr<const IHexapodCoordinateConverter>& converter_ptr);


    //! @brief ノード間を補間する．
    //! @param[in] node 現在のノード．
    //! @param[in] next_node 次のノード．
    //! @return 補間されたノード．
    std::vector<RobotStateNode> CreateInterpolatedNode(const RobotStateNode& node,
                                                       const RobotStateNode& next_node) const;

private:
    static constexpr int kGroundInterpolatedNodeNum = 10;  //!< 脚が接地する際の補間ノード数．
    static constexpr int kBodyMoveInterpolatedNodeNum = 10;  //!< 胴体が移動する際の補間ノード数．
    static constexpr int kFreeInterpolatedNodeNum = 10;  //!< 脚が遊脚する際の補間ノード数．

    //! @return 補間が必要ないならば trueを返す．
    bool IsNoChange(const RobotStateNode& current_node, const RobotStateNode& next_node) const;

    //! @return 胴体が移動するなら trueを返す．
    bool IsBodyMove(const RobotStateNode& current_node, const RobotStateNode& next_node) const;

    //! @return 胴体が回転するなら trueを返す．
    bool IsBodyRot(const RobotStateNode& current_node, const RobotStateNode& next_node) const;

    std::vector<RobotStateNode> CreateBodyMoveInterpolatedNode(
        const RobotStateNode& current_node, const RobotStateNode& next_node) const;

    std::vector<RobotStateNode> CreateBodyRotInterpolatedNode(
        const RobotStateNode& current_node, const RobotStateNode& next_node) const;

    //! @brief 接地動作をする脚の indexを取得する．
    std::vector<int> GetGroundMoveIndex(const RobotStateNode& current_node,
                                        const RobotStateNode& next_node) const;

    //! @brief 遊脚動作をする脚の indexを取得する．
    std::vector<int> GetFreeMoveIndex(const RobotStateNode& current_node,
                                      const RobotStateNode& next_node) const;

    std::vector<RobotStateNode> CreateLegMoveInterpolatedNode(
        const RobotStateNode& current_node, const RobotStateNode& next_node) const;


    const std::shared_ptr<const IHexapodCoordinateConverter> converter_ptr_;
};

}  // namespace designlab


#endif  // DESIGNLAB_INTERPOLATED_NODE_CREATOR_H_
