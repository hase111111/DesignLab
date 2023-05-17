#pragma once
#include <boost\thread.hpp>
#include "vectorFunc.h"
#include "hexapod.h"
#include "Define.h"

class mapData
{
public:
	mapData(int in_mapData_Max);
	~mapData() = default;
	int getSize();
	void operator()();					//これがメインとなる関数
	myvector::SVector* mapDatafront;	//セマフォで守られている方リソースを占拠する時間短縮のためbackを更新frontにcopy
	myvector::SVector* center_of_mass;	//重心位置　デバックでしか使わないかも
	float *thP, *thR, *thY;				//自機の向き 
	boost::mutex* mtxMapData;			//ミューテックス Data通信用
	void SetdummyMapData();
	bool *isCicle;						//メインの関数を抜ける時0

private:
	const int mapData_Max;				//歩容生成部と共有するmapDataの個数
	myvector::SVector* mapDataBack;
	myvector::SVector rowMapData[Define::MAPDATA3D_MAX];
	Hexapod phantomX;
};
