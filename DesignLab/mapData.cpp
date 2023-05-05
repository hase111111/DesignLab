#include "mapData.h"

mapData::mapData(int in_mapData_Max) : mapData_Max(in_mapData_Max)
{
	//mapDatafront = new myvector::VECTOR[ mapData_Max ];
	mapDataBack = new myvector::SVector[ mapData_Max ];
	std::cout<<"new";

}

int mapData::getSize(){
	return mapData_Max;
}


void mapData::operator()(){

	myvector::SVector comNew, comOld = myvector::VGet(0.0, 0.0, 100.0);
	myvector::SVector comDiff;		//前回からの重心移動量
	float thpNew, thrNew, thyNew, thpOld = 0, thrOld = 0, thyOld = 0;
	float thpDiff, thrDiff, thyDiff;
	float thpAdd = 0, thrAdd = 0, thyAdd = 0;

	while(*isCicle){
		//画像データを入手，3次元位置を計算，歩容生成に必要なデータをmapDataBackに代入，mapDataBackをmapDatafrontにコピー
		{
			boost::mutex::scoped_lock lk(*mtxMapData); //排他制御が必要
			comNew = *center_of_mass;
			thpNew = *thP;
			thrNew = *thR;
			thyNew = *thY;

		}
		comDiff = myvector::VSub(comNew, comOld);
		thpDiff = thpNew - thpOld;
		thrDiff = thrNew - thrOld;
		thyDiff = thyNew - thyOld;
		comOld = comNew;
		thpOld = thpNew;
		thrOld = thrNew;
		thyOld = thyNew;
		thpAdd +=thpDiff;
		thrAdd +=thrDiff;
		thyAdd +=thyDiff;


		for(int i = 0; i < mapData_Max; i++){
			rowMapData[i] = myvector::VSub(rowMapData[i], comDiff);
			//rowMapData[i] = myvector::VRot_R(myvector::VRot_R(rowMapData[i],myvector::VGet(0.0, 0.0, 0.0), -thpDiff, -thyDiff, -thrDiff),myvector::VGet(0.0, 0.0, 0.0), thpDiff, thrDiff, thyDiff);
			//rowMapData[i] = myvector::VRot(myvector::VRot_R(rowMapData[i],myvector::VGet(0.0, 0.0, 0.0), -thpDiff, -thyDiff, -thrDiff),myvector::VGet(0.0, 0.0, 0.0), thpDiff, thrDiff, thyDiff);
			mapDataBack[i] = myvector::VRot_R(rowMapData[i],myvector::VGet(0.0, 0.0, 0.0), -thpAdd, -thrAdd, -thyAdd);
			//rowMapData[i] = myvector::VRot(rowMapData[i],myvector::VGet(0.0, 0.0, 0.0), thpDiff, thrDiff, thyDiff);
			//mapDataBack[i] = myvector::VRot(rowMapData[i],myvector::VGet(0.0, 0.0, 0.0), thpNew, thrNew, thyNew);
			//mapDataBack[i] = rowMapData[i];
		}

		{
			boost::mutex::scoped_lock lk(*mtxMapData); //排他制御が必要
			memcpy(mapDatafront,mapDataBack,sizeof(myvector::SVector) * mapData_Max);
		}
	}
	std::cout<<"mapData終了\n";
}


void mapData::SetdummyMapData()
{
	phantomX.setMyPosition(myvector::VGet(0,100,0));
	std::cout<<phantomX.showGlobalMyPosition().x<<","<<phantomX.showGlobalMyPosition().y<<","<<phantomX.showGlobalMyPosition().z<<"\n";
	phantomX.setMyDirection(*thP, *thR, *thY);
	myvector::SVector IPosition_of_Leg[6];
	IPosition_of_Leg[0] = myvector::VGet(120,  80, -100);
	IPosition_of_Leg[1] = myvector::VGet(120,   0, -100);
	IPosition_of_Leg[2] = myvector::VGet(120, -80, -100);
	IPosition_of_Leg[3] = myvector::VGet(-120, -80, -100);
	IPosition_of_Leg[4] = myvector::VGet(-120,   0, -100);
	IPosition_of_Leg[5] = myvector::VGet(-120,  80, -100);


	phantomX.setMyLegPosition(IPosition_of_Leg);

	for(int i = 0; i < 6; i++){
		rowMapData[i] = phantomX.showGlobalLegPosition(i);
		std::cout<<rowMapData[i].x<<","<<rowMapData[i].y<<","<<rowMapData[i].z<<"\n";
	}


	for(int i = 6; i < Define::MAPDATA3D_MAX - 6; i+=6){

	if(i > 90 && i < 103)continue;
		rowMapData[i].x =  - 200;
		rowMapData[i].y = i * 10 - 500;
		rowMapData[i].z = -100;

		rowMapData[i+1].x =  - 150;
		rowMapData[i+1].y = i * 10 - 500;
		rowMapData[i+1].z = -100;

		rowMapData[i+2].x = - 250;
		rowMapData[i+2].y = i * 10 - 500;
		rowMapData[i+2].z = -100;

//	if(i > 90 && i < 112)continue;

		rowMapData[i+3].x = 400  - 200;
		rowMapData[i+3].y = i * 10 - 500;
		rowMapData[i+3].z = -100;

		rowMapData[i+4].x = 400 - 150;
		rowMapData[i+4].y = i * 10 - 500;
		rowMapData[i+4].z = -100;

		rowMapData[i+5].x = 400 - 250;
		rowMapData[i+5].y = i * 10 - 500;
		rowMapData[i+5].z = -100;
	}

	//for(int i = 6; i < MAPDATA3D_MAX; i++){
	//	//----------------------------ランダム-------------------------------------
	//	rowMapData[i].x = (rand()%4000)-1500;				//脚着地可能点
	//	rowMapData[i].y = (rand()%4000)-500;				//脚着地可能点
	//	rowMapData[i].z = -100;
	//	//-------------------------------------------------------------------------
	//
	//	//rowMapData[i].x = 400 * (i%2) - 200;
	//	//rowMapData[i].y = i * 30 - 500;
	//	//rowMapData[i].z = -100;
	//}
}
