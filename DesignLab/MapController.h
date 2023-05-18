#pragma once
#include "MapConst.h"
#include <vector>
#include "vectorFunc.h"
#include "listFunc.h"

// SPLP�Ŏg�p�D�^����ꂽ���W����u���b�N�ԍ������߂� p1:�l�p�`�G���A�̍����̓_ p2:�E��̓_�@�@x1:�ŏ��@x2:�ő�@y1:�ŏ��@y2:�ő�
void AreaDivide(const myvector::SVector& p1, const myvector::SVector& p2, int& x1, int& x2, int& y1, int& y2);

// main.cpp
void MapSqrtDivide(const myvector::SVector mapData[MapConst::MAPDATA3D_MAX], std::vector< std::vector< std::vector<myvector::SVector> > >& divideMapData, int pointNum[MapConst::LP_DIVIDE_NUM][MapConst::LP_DIVIDE_NUM]);

//�}�b�v�̕��s�ړ� main.cpp
void recalMap(myvector::SVector p_mapData3D[MapConst::MAPDATA3D_MAX], const LNODE& _current_condition, const  LNODE& _past_condition);
