﻿//! @file interface_dxlib_node_setter.h
//! @brief DxLibのGuiやRendererでノードのセットを行うためのインターフェース．

#ifndef DESIGNLAB_INTERFACE_DXLIB_NODE_SETTER_H_
#define DESIGNLAB_INTERFACE_DXLIB_NODE_SETTER_H_

#include "robot_state_node.h"


//! @class IDxlibNodeSetter
//! @brief DxLibのGuiやRendererでノードのセットを行うためのインターフェース．
class IDxlibNodeSetter
{
public:
	virtual ~IDxlibNodeSetter() = default;

	//! @brief ノードをセットする．
	//! @param node ノード番号．
	virtual void SetNode(const RobotStateNode& node) = 0;

};

#endif // DESIGNLAB_INTERFACE_DXLIB_NODE_SETTER_H_
