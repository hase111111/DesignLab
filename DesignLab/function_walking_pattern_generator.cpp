#define _CRT_SECURE_NO_WARNINGS
#include"function_walking_pattern_generator.h"
#include <iostream>

//ファイルに書き込んである物の説明
void Description_txt( std::ofstream& foutKaisetu){

	foutKaisetu<<"route_A"<<"\n";
	foutKaisetu<<"選択したルートの詳細な情報"<<"\n\n";

	foutKaisetu<<"route_A_D"<<"\n";
	foutKaisetu<<"重心移動区切りでの求めたルート"<<"\n\n";

	foutKaisetu<<"route_B"<<"\n";
	foutKaisetu<<"route_Aのサイズ、"<<"\n\n";

	foutKaisetu<<"route_B_D"<<"\n";
	foutKaisetu<<"毎回の探索によって出てきたルートのすべて"<<"\n\n";

	foutKaisetu<<"route_C"<<"\n";
	foutKaisetu<<"求めたルート"<<"\n\n";

	foutKaisetu<<"route_C_D"<<"\n";
	foutKaisetu<<"求めたルートのkaisou情報のみ"<<"\n\n";

	foutKaisetu<<"route_D"<<"\n";
	foutKaisetu<<"次の探索に引き継ぐ情報、次の初期値"<<"\n\n";

	foutKaisetu<<"route_D_D"<<"\n";
	foutKaisetu<<"マップ情報"<<"\n\n";

	foutKaisetu<<"route_A1"<<"\n";
	foutKaisetu<<""<<"\n\n";

	foutKaisetu<<"route_A2"<<"\n";
	foutKaisetu<<""<<"\n\n";

	foutKaisetu<<"route_A3"<<"\n";
	foutKaisetu<<""<<"\n\n";

	foutKaisetu<<"route_A4"<<"\n";
	foutKaisetu<<""<<"\n\n";

	foutKaisetu<<"route_A5"<<"\n";
	foutKaisetu<<""<<"\n\n";

	foutKaisetu<<"route_A6"<<"\n";
	foutKaisetu<<""<<"\n\n";
}

//現在時刻をコンソールとファイルに出力
int now_time(std::ofstream& fout_log){
	time_t now = time(NULL);
	 struct tm *pnow = localtime(&now);
	char buff[128]="";
	sprintf(buff,"\t%d:%d:%d",pnow->tm_hour,pnow->tm_min,pnow->tm_sec);
	printf(buff);
	fout_log<<buff;
	std::cout<<"\t"<<(double)clock()<<"\n";
	fout_log<<"\t"<<(double)clock()<<"\n";
	return 0;
}			//標準入出力とlogに書き込み


//現在時刻をコンソールに出力
int now_time(){
	time_t now = time(NULL);
	 struct tm *pnow = localtime(&now);
	char buff[128]="";
	sprintf(buff,"\t%d:%d:%d",pnow->tm_hour,pnow->tm_min,pnow->tm_sec);
	printf(buff);
	std::cout<<"\t"<<(double)clock()<<"\t"<<(double)clock()/1000/60<<"m\n";
	return 0;
}			//標準入出力とlogに書き込み

//ログを出力する
void output_log(const char *str, std::ofstream& fout_log){
	std::cout<<str;
	fout_log<<str;
	now_time(fout_log);
}

//進行方向を与える関数ダミー
void getWalkingDirection(myvector::SVector* walkingDirection){

	*walkingDirection = myvector::VGet(0, 1, 0);

}

//マップ情報を与える関数ダミー
void getMap(myvector::SVector *&p_mapData3D, int* mapData3D_MAX, myvector::SVector legPosi[6], myvector::SVector center_of_mass){
	//std::cout<<"mapData3D_MAX"<<"\n";

	//*mapData3D_MAX = 500;
	//*mapData3D_MAX = 166;
	*mapData3D_MAX = 1000;
	
	p_mapData3D = new myvector::SVector[ *mapData3D_MAX ];


	for(int i = 6; i < *mapData3D_MAX-6; i+=6){

	if(i > 90 && i < 109)continue;		
		p_mapData3D[i].x =- 200;
		p_mapData3D[i].y = i * 10 - 500;
		p_mapData3D[i].z = 0;

		p_mapData3D[i+1].x =- 150;
		p_mapData3D[i+1].y = i * 10 - 500;
		p_mapData3D[i+1].z = 0;

		p_mapData3D[i+2].x =- 250;
		p_mapData3D[i+2].y = i * 10 - 500;
		p_mapData3D[i+2].z = 0;

//	if(i > 90 && i < 112)continue;

		p_mapData3D[i+3].x = 400 - 200;
		p_mapData3D[i+3].y = i * 10 - 500;
		p_mapData3D[i+3].z = 0;

		p_mapData3D[i+4].x = 400 - 150;
		p_mapData3D[i+4].y = i * 10 - 500;
		p_mapData3D[i+4].z = 0;

		p_mapData3D[i+5].x = 400 - 250;
		p_mapData3D[i+5].y = i * 10 - 500;
		p_mapData3D[i+5].z = 0;

	}



	//for(int i = 6; i < *mapData3D_MAX; i++){
	//	//----------------------------ランダム-------------------------------------
	//	//p_mapData3D[i].x = (rand()%4000) - 1300;				//脚着地可能点
	//	//p_mapData3D[i].y = (rand()%2000) - 1000;				//脚着地可能点
	//	//p_mapData3D[i].z = 0;
	//	//-------------------------------------------------------------------------

	//	p_mapData3D[i].x = 400 * (i%2) - 200;
	//	p_mapData3D[i].y = i * 30 - 500;
	//	p_mapData3D[i].z = 0;
	//	
	//	//-------------------------------------------------------------------------

	//}


	//for(int i = 6; i < *mapData3D_MAX; i++){
	//	//----------------------------ランダム-------------------------------------
	//	p_mapData3D[i].x = (rand()%4000) - 1500;				//脚着地可能点
	//	p_mapData3D[i].y = (rand()%4000) - 500;				//脚着地可能点
	//	p_mapData3D[i].z = 0;
	//	//-------------------------------------------------------------------------

	//	//p_mapData3D[i].x = 400 * (i%2) - 200;
	//	//p_mapData3D[i].y = i * 30 - 500;
	//	//p_mapData3D[i].z = 0;
	//	
	//	//-------------------------------------------------------------------------

	//}


	

	Hexapod phantomX;
	phantomX.setMyPosition(center_of_mass);
	phantomX.setMyDirection(0,0,0);
	phantomX.setLocalLeg2Pos(legPosi);

	p_mapData3D[0] = myvector::addVec(legPosi[0], phantomX.getGlobalCoxaJointPos(0));
	p_mapData3D[1] = myvector::addVec(legPosi[1], phantomX.getGlobalCoxaJointPos(1));
	p_mapData3D[2] = myvector::addVec(legPosi[2], phantomX.getGlobalCoxaJointPos(2));
	p_mapData3D[3] = myvector::addVec(legPosi[3], phantomX.getGlobalCoxaJointPos(3));
	p_mapData3D[4] = myvector::addVec(legPosi[4], phantomX.getGlobalCoxaJointPos(4));
	p_mapData3D[5] = myvector::addVec(legPosi[5], phantomX.getGlobalCoxaJointPos(5));
	for(int i = 0; i < 6; i++)myvector::VectorOutPut(p_mapData3D[i]);
	for(int i = 0; i < 6; i++)myvector::VectorOutPut(phantomX.getGlobalLeg2Pos(i));

}

myvector::SVector VCangeBodyToLeg(myvector::SVector& Vin, float thP, float thR, float thY){
	return myvector::VRot(myvector::VGet(Vin.x, -Vin.y, -Vin.z), thP, thR, thY - 3.14/2);  // - 3.14/2はどうにかならないかな
}