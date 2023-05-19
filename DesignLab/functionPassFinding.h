#pragma once

#include "vectorFunc.h"
#include "listFunc.h"
#include "PassFinding.h"
#include "MapState.h"

//複数スレッドによる探索・結果を返す
LNODE functionPassFinding(const LNODE& _current_condition, const  LNODE& _past_condition, const STarget Target, const MapState* _p_map_state, LNODE pass_root[100], int& m_node_num);

//ノードを2つ比較して，脚状態と重心位置がほぼ同じならtrueを返す
bool isEqualNode(LNODE node1,LNODE node2);
