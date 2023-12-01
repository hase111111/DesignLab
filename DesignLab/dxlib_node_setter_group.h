﻿//! @file dxlib_node_setter_group.h
//! @brief DxLibでノードの設定を行うクラスをまとめるクラス．

#ifndef DESIGNLAB_DXLIB_NODE_SETTER_GROUP_H_
#define DESIGNLAB_DXLIB_NODE_SETTER_GROUP_H_

#include <memory>
#include <vector>

#include "interface_dxlib_node_setter.h"
#include "robot_state_node.h"


//! @class DxlibNodeSetterGroup
//! @brief DxLibでノードの設定を行うクラスをまとめるクラス．
class DxlibNodeSetterGroup final
{
public:

	//! @brief ノードの設定を行うクラスを登録する．
	//! @param [in] setter ノードの設定を行うクラス．
	void Register(const std::shared_ptr<IDxlibNodeSetter>& setter);

	//! @brief 登録済みの全てのクラスに対してノードの設定を行う．
	//! @param [in] node ノード．
	void SetNode(const RobotStateNode& node);

private:

	std::vector<std::shared_ptr<IDxlibNodeSetter>> setters_;

};

#endif // DESIGNLAB_DXLIB_NODE_SETTER_GROUP_H_