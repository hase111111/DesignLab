//! @file interface_node_creator_builder
//! @brief INodeCreatorを生成するためのビルダークラス

#ifndef DESIGNLAB_INTERFACE_NODE_CREATOR_BUILDER_H_
#define DESIGNLAB_INTERFACE_NODE_CREATOR_BUILDER_H_

#include <map>
#include <memory>

#include "abstract_hexapod_state_calculator.h"
#include "devide_map_state.h"
#include "hexapod_next_move.h"
#include "interface_node_creator.h"
#include "map_state.h"


//! @class INodeCreatorBuilder
//! @brief INodeCreatorを生成するためのビルダークラス
class INodeCreatorBuilder
{
public:

	INodeCreatorBuilder() = default;
	virtual ~INodeCreatorBuilder() = default;

	//! @brief INodeCreatorを生成する
	//! @param [in] map 分割されたマップ
	//! @param [in] calculator_ptr ロボットの状態を計算するクラス
	//! @param [out] node_creator INodeCreatorを格納するmap
	//! @n key:HexapodMove, value:INodeCreator
	//! @n つまり，ロボットの動作に対応するINodeCreatorを格納する必要がある
	virtual void Build(const DevideMapState& map, const std::shared_ptr<const AbstractHexapodStateCalculator>& calculator_ptr,
		std::map<HexapodMove, std::unique_ptr<INodeCreator> > *node_creator) const = 0;
};


#endif