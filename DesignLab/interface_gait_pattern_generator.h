﻿//! @file interface_pass_finder.h
//! @brief グラフ探索による歩容パターン生成を行うクラスのインターフェース．

#ifndef DESIGNLAB_INTERFACE_GAIT_PATTERN_GENERATOR_H_
#define DESIGNLAB_INTERFACE_GAIT_PATTERN_GENERATOR_H_

#include <vector>

#include "graph_search_result_recoder.h"
#include "map_state.h"
#include "robot_state_node.h"
#include "target_robot_state.h"


//! @class IGaitPatternGenerator
//! @brief グラフ探索による歩容パターン生成を行うクラスのインターフェース．
//! @details 
//! @n 波東さんのプログラムで言うところのPassFindingクラス．
//! @n 実体は作成できないのでこれを継承してたクラスを使うこと．
//! @n 継承をするクラスのデストラクタはvirtualにしておく．
//! @n 参考 https://www.yunabe.jp/docs/cpp_virtual_destructor.html
class IGaitPatternGenerator
{
public:

	IGaitPatternGenerator() = default;
	virtual ~IGaitPatternGenerator() = default;


	//! @brief グラフ探索を行い，次の動作として最適なノードを返す．
	//! @param [in] current_node 現在のロボットの状態を表すノード．親ノードを渡す必要がある．
	//! @param [in] map 現在のマップの状態．
	//!	@param [in] target 目標．
	//! @param [out] output_node 結果のノード．
	//! @return GraphSearchResult グラフ探索の結果を返す．成功か失敗か．
	virtual GraphSearchResult GetNextNodebyGraphSearch(
		const RobotStateNode& current_node, 
		const MapState& map, 
		const TargetRobotState& target, 
		RobotStateNode* output_node
	) = 0;
};


#endif	// DESIGNLAB_INTERFACE_GAIT_PATTERN_GENERATOR_H_