#pragma once

#include "pch.h"
#include "vectorFunc.h"
#include "listFunc.h"
#include "PassFinding.h"
#include "mainfunction.h"



//複数スレッドによる探索・結果を返す
LNODE functionPassFinding(LNODE CurrentCondition, LNODE PastCondition, int mapData3D_MAX, myvector::SVector* mapData3D, STarget Target, LNODE pass_root[100], std::vector< std::vector< std::vector<myvector::SVector> > > &divideMapData, int pointNum[LP_DIVIDE_NUM][LP_DIVIDE_NUM], int &m_node_num);

//ノードを2つ比較して，脚状態と重心位置がほぼ同じならtrueを返す
bool isEqualNode(LNODE node1,LNODE node2);

bool isEqualbackupNode(LNODE node1);

void writeFileMapData3D(myvector::SVector *p_mapData3D, int mapData3D_MAX);

void FILEOPEN();

void FILECLOSE();