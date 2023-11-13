﻿//! @file pass_finder_basic.h
//! @brief 普通にグラフ探索を行い，歩容パターン生成を行うクラス

#ifndef DESIGNLAB_PASS_FINDER_BASIC_H_
#define DESIGNLAB_PASS_FINDER_BASIC_H_

#include "interface_pass_finder.h"

#include <memory>
#include <vector>

#include "interface_graph_searcher.h"
#include "graph_tree_creator.h"
#include "robot_state_node.h"


//! @class PassFinderBasic
//! @brief 普通にグラフ探索を行い，歩容パターン生成を行うクラス
class PassFinderBasic final : public IPassFinder
{
public:

	//! @param[in] graph_tree_creator グラフ探索を行う木構造のグラフを作成するクラス．unique_ptrで渡す．
	//! @param[in] graph_searcher グラフ探索を行うクラス．unique_ptrで渡す．
	PassFinderBasic(
		std::unique_ptr<GraphTreeCreator>&& graph_tree_creator_ptr, 
		std::unique_ptr<IGraphSearcher>&& graph_searcher_ptr
	);

	~PassFinderBasic() = default;


	GraphSearchResult GetNextNodebyGraphSearch(const RobotStateNode& current_node, const MapState& map_ref, const TargetRobotState& target, RobotStateNode* output_node) override;

private:

	const std::unique_ptr<GraphTreeCreator> graph_tree_creator_ptr_;	//!< グラフ探索を行う木構造のグラフを作成するクラス

	const std::unique_ptr<IGraphSearcher> graph_searcher_ptr_;		//!< グラフ探索を行うクラス

	std::vector<RobotStateNode> graph_tree_;
};


#endif  // DESIGNLAB_PASS_FINDER_BASIC_H_