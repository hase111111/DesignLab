#pragma once
#include "MapState.h"
#include "Node.h"

class ComMoveNodeCreator final
{
public:

	//! @brief 重心を平行移動したノードを生成する
	//! @param[in] _current_node 重心を平行移動するノード
	//! @param[in] _current_num 重心を平行移動するノードの番号
	//! @param[out] _output_graph 重心を平行移動したノードを格納するコンテナ
	void create(const SNode& _current_node, const int _current_num, std::vector<SNode>& _output_graph);

private:

	const MapState* mp_Map;
};
