#pragma once

#include "vectorFunc.h"
#include "listFunc.h"
#include "PassFinding.h"

//複数スレッドによる探索・結果を返す
LNODE functionPassFinding(const LNODE& _current_condition, const  LNODE& _past_condition, STarget Target, LNODE pass_root[100], std::vector< std::vector< std::vector<myvector::SVector> > >& divideMapData, int pointNum[MapConst::LP_DIVIDE_NUM][MapConst::LP_DIVIDE_NUM], int& m_node_num);

//ノードを2つ比較して，脚状態と重心位置がほぼ同じならtrueを返す
bool isEqualNode(LNODE node1,LNODE node2);
