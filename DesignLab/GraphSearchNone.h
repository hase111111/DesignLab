#pragma once
#include "MapState.h"
#include "Node.h"
#include "InterfaceGraphSearch.h"
#include <memory>

class GraphSearchNone final :public IGraphSearch
{
public:
	GraphSearchNone() = default;
	~GraphSearchNone() = default;

	bool getNextNodebyGraphSearch(const SNode& _current_node, const MapState* const _p_map, const STarget& _target, SNode& _output_node) override;

private:

};


//! @file GraphSearchNone.h 
//! @brief グラフ探索を行わないクラスの実装．
//! @author 長谷川

//! @class GraphSearchNone
//! @brief グラフ探索を行わないクラス．Graphicクラスのデバッグを行う際に用いる．
//! @author 長谷川