#pragma once
#include "vectorFunc.h"
#include "listFunc.h"
#include "mapData.h"

#define READMAPDATA_FROM_CSV //csvファイル（excelファイル）からマップデータを読み取るときはdefine
#define FOOT_HOLD_XY_DIST 20	//z軸からみたときの足場（脚接地可能点）の間隔[mm]、格子点状
#define START_ROUGH_TARRAIN_Y 400 //直進移動のときの、不整地と水平面の境界[mm]

//#define SLOPE	//斜面を不整地とするときdefine
#define THETA_SLOPE	20	//最大傾斜角[°]	斜面の最大傾斜角
#define XI_SLOPE	(double) 0//最大傾斜方位角ξ[°]　y軸(グローバル)と最大傾斜方向へのベクトル（勾配ベクトル）のなす角 ±90°は89.9くらいにしとく

#define STEP	//階段を吸整地とするときはdefine
#define HEIGHT_STEP	-140	//段差高さ[mm]
#define DEPTH_STEP	500	//奥行き[mm]

//#define ISOSELES_TRIANGLE	//2等辺三角形の不整地にするときはdefine
#define WIDE_TRI	400		//2等辺三角形の底辺の長さ/2[mm]
#define DEPTH_TRI	400		//2等辺三角形の奥行き
#define THETA_TRI	10		//底角[°]

#define SQUARE_SIZE 100 //＝FOOT_HOLD_XY_DIST*n(n=1,2,3...)n=1なら1点ずつ、2なら4点、nならn^2点の正方形が対象になる正方形の一片の長さ[mm]
#define HOLE_RANDOM
#define HOLE_RATE 60		//不整地上の足場を除外する割合。ホール率[%]
#define HEIGHT_MAGNIFICATION 10	//高さ方向のランダムな倍率と刻み数[-]　ここ後で名前変える、有りそうだから
//#define RANDOM_ADD_Z 3		// ±MAGNIFICATION(0~n)←乱数×RANDOM_ADD_Z分だけ高さ方向に加える。高さ[mm] 高さ変えないときは、CO
#define START_RANDOM_R 200	//初期位置のランダムで配置される範囲、半径[mm]。
#define INVALID_FOOT_HOLD -10000 //ホール部にあった脚設置可能点を飛ばす座標

#ifndef LP_DIVIDE_NUM
#define LP_DIVIDE_NUM 40	//脚接地可能点を平方分割する際の１辺の分割数
#endif
#ifndef MAP_X_MIN
#define MAP_X_MIN -1000
#endif
#ifndef MAP_X_MAX
#define MAP_X_MAX 1000
#endif
#ifndef MAP_Y_MIN
#define MAP_Y_MIN -400
#endif
#ifndef MAP_Y_MAX
#define MAP_Y_MAX 2600
#endif






myvector::SVector LNODEGCOMtoD_L_Pp_comm(myvector::SVector VIn);
myvector::SVector LNODERPtoD_L_Pp_robotPosture(myvector::SVector VIn);

void getMap(myvector::SVector *p_mapData3D, int* mapData3D_MAX, LNODE *CurrentCondition, const int f);

void recalMap(myvector::SVector *p_mapData3D, int mapData3D_MAX, LNODE *CurrentCondition, LNODE *PastCondition);

void startMap(myvector::SVector *allmapData, myvector::SVector *mapData, int mapData3D_MAX);

void DetectionPoint(myvector::SVector *allmapData, myvector::SVector *mapData, int mapData3D_MAX, LNODE* CurrentCondition);

int ReadMapDataFromFile(const std::string& filename, myvector::SVector *map, const char delimiter = ',');

void WriteMapDataToFile(std::ofstream& filename, const myvector::SVector *map, const int* mapData3D_MAX);

void MapSqrtDivide(myvector::SVector *mapData, int mapDataNum, std::vector< std::vector< std::vector<myvector::SVector> > > &divideMapData, int pointNum[LP_DIVIDE_NUM][LP_DIVIDE_NUM]);

void AreaDivide(myvector::SVector p1, myvector::SVector p2, int &x1, int &x2, int &y1, int &y2);
void ReadStartNodeFromFile(LNODE &node, const int f,const char delimiter = ',');
void WriteStartNodeToFile(const LNODE& node,const int f);
void SetConditionForStripe(LNODE &node,const int f);

void SquareMap(myvector::SVector *mapData, int x, int y, int nMap);
void SquareMap(myvector::SVector  *mapData, int x, int y, int nMap, int seed);
void CircleMap(myvector::SVector *mapData, myvector::SVector center, int Rmin, int Rmax, float amax, float amin, int nMap, int seed);
void CircleMap(myvector::SVector *mapData, myvector::SVector center, int Rmin, int Rmax, float amax, float amin, int nMap);
void GetRandom(int mapDataNum, int min, int max, int* Random);//マップデータの数、乱数の最小値、乱数の最大値、乱数を格納する配列[マップデータの数]