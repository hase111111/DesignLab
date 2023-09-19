//! @file interface_node_creator.h
//! @brief ノード生成インターフェース

#ifndef DESIGNLAB_INTERFACE_NODE_CREATOR_H_
#define DESIGNLAB_INTERFACE_NODE_CREATOR_H_


#include <vector>

#include "node.h"


//! @class INodeCreator
//! @brief ノード生成インターフェース
class INodeCreator
{
public:

	//! @brief コンストラクタでは次動作を設定する．またマップのポインタを受け取る
	INodeCreator() = default;
	virtual ~INodeCreator() = default;


	//! @brief 現在のノードから次のノード群を生成する
	//! @param[in] current_node 現在のノード
	//! @param[in] current_node_index 現在のノードのインデックス
	//! @param[out] output_graph 生成したノード群を返す
	virtual void Create(const SNode& current_node, int current_node_index, std::vector<SNode>* output_graph) = 0;
};


#endif // DESIGNLAB_INTERFACE_NODE_CREATOR_H_