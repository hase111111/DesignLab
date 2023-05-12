#include "mainfunction.h"
#include "hexapod.h"
#include <iostream>
#include <random>

myvector::SVector LNODEGCOMtoD_L_Pp_comm(myvector::SVector VIn){

	return myvector::VGet(VIn.y, VIn.x,VIn.z);

}
myvector::SVector LNODERPtoD_L_Pp_robotPosture(myvector::SVector VIn){

	return myvector::VGet(VIn.y, VIn.x,VIn.z);

}

//マップ情報を与える関数ダミー
void getMap(myvector::SVector *p_mapData3D, int* mapData3D_MAX, LNODE* CurrentCondition, const int f) 
{
	//std::cout<<"mapData3D_MAX"<<"\n";
	*mapData3D_MAX = (MAP_X_MAX -MAP_X_MIN)/FOOT_HOLD_XY_DIST * (MAP_Y_MAX - MAP_Y_MIN)/FOOT_HOLD_XY_DIST;
	//脚接地候補点の数(変更する場合)最大値はmapData.h参照
	//上式の定数の値を変えた場合は、mapData.hのMAPDATA3D_MAXも同値になるように変更すること。ちょいめんどいけど
	//*mapData3D_MAX = 1000;
	//*mapData3D_MAX = 100;




	Hexapod phantomX;
	phantomX.setMyPosition(CurrentCondition->global_center_of_mass);//重心位置グローバル
	//phantomX.setTravelingDirection(CurrentCondition->global_center_of_mass);//進行方向(重心位置(0,0,110))
	phantomX.setMyDirection(CurrentCondition->pitch, CurrentCondition->roll, CurrentCondition->yaw);//姿勢(テイトブライアン角)グローバル、初期姿勢が違うときは変更する必要あり//(0,0,0)から変更
	phantomX.setLocalLegPos(CurrentCondition->Leg);
	phantomX.setLocalLeg2Pos(CurrentCondition->Leg2);//付け根から脚先の位置ローカル
	//初期姿勢における足先座標を脚設置可能点とする
	//p_mapData3D[0] = phantomX.getGlobalLeg2Pos(0);
	//p_mapData3D[1] = phantomX.getGlobalLeg2Pos(1);
	//p_mapData3D[2] = phantomX.getGlobalLeg2Pos(2);
	//p_mapData3D[3] = phantomX.getGlobalLeg2Pos(3);
	//p_mapData3D[4] = phantomX.getGlobalLeg2Pos(4);
	//p_mapData3D[5] = phantomX.getGlobalLeg2Pos(5);
	p_mapData3D[0] = phantomX.getGlobalLegPos(0);
	p_mapData3D[1] = phantomX.getGlobalLegPos(1);
	p_mapData3D[2] = phantomX.getGlobalLegPos(2);
	p_mapData3D[3] = phantomX.getGlobalLegPos(3);
	p_mapData3D[4] = phantomX.getGlobalLegPos(4);
	p_mapData3D[5] = phantomX.getGlobalLegPos(5);
	//for (int i = 0; i < 6; i++) p_mapData3D[i].z = 0;
	for (int i = 0; i < 6; i++) {
		if (p_mapData3D[i].z >= 25) {
			p_mapData3D[i].x = 10000;
		}
	}
	for (int i = 0; i < 6; i++)myvector::VectorOutPut(p_mapData3D[i]);
	for (int i = 0; i < 6; i++)myvector::VectorOutPut(phantomX.getGlobalLeg2Pos(i));


	//p_mapData3D = new myvector::VECTOR[ *mapData3D_MAX ];

	//円形に配置
	//myvector::VECTOR center = { -1000,0,0 };	//円の中心
	//CircleMap(p_mapData3D, center, 600, 1400, 180.0, -180.0, *mapData3D_MAX);		//完全ランダム
	//CircleMap(p_mapData3D, center, 600, 1400, 180.0, -180, *mapData3D_MAX, 4);		//同じシミュレーション繰り返したいとき

		//四角形内にランダムに生成　実行ごとに異なる
	//SquareMap(p_mapData3D, 1000, 6000, *mapData3D_MAX);


	//直線　同じシミュレーションを繰り返したいとき
		//SquareMap(p_mapData3D, 1000, 6000, *mapData3D_MAX, 3);

	////脚接地点の座標グローバル
	//まっすぐ整地
	//for(int i = 6; i < *mapData3D_MAX-20; i+=10){
	//	for(int j = 0; j < 20; j++){
	//		p_mapData3D[i+j].x = -380 + 80 * j;
	//		p_mapData3D[i+j].y = i * 10 - 380;
	//		p_mapData3D[i+j].z = 0;
	//	}
	//}

	/*////////////////////////////////////////////////////////////////////////////
	個人的な計算用めも	190613時点　phantomX前提
	脚を伸ばすと脚長は大体190mm	(hexapod.h参照)
	重心高さは110mm
	脚高さは80mmだから，遊脚した脚先は地面からは30mmの位置
	有効な高さ可動範囲は80mm~190mm
	余裕をもって90~170にすべきか
	その高さにおける最大到達半径はhexapodクラスのLegROM_r参照
	最小半径は50固定
	*/////////////////////////////////////////////////////////////////////////////



	//①平面から斜面（斜度10度＝勾配約17.6%）への乗り移り。40mm間隔で設置可能点を生成。
	//日本の坂道の最大勾配は37%=20°ぐらい。
	//べた踏み坂ってやつは大体6%=3~4°くらいらしい。

	//これは40mm間隔の格子点マップデータは2000くらいでやる。
	//for (int i = 6; i < *mapData3D_MAX -20; i += 20) {//*mapData3D_MAX はmapData.h
	//	for (int j = 0; j < 20; j++) {
	//		p_mapData3D[i + j].x = -380 + 40 * j;//x軸方向に40mm間隔でプロット -380mm<=x<=380mm
	//		p_mapData3D[i + j].y = (i - 6) * 2 - 220;//y軸方向に40mm間隔でプロット -220mm<=y<=1760mm
	//		p_mapData3D[i + j].z = 0;
	//		if (i > *mapData3D_MAX / 10) {
	//			
	//			p_mapData3D[i + j].z = -tan(10 * 3.1415926 / 180)*(i - (*mapData3D_MAX / 10) - 6) * 2;//普通にz=tan(theta)*y
	//		}
	//	}
	//}

	//これは20mm間隔の格子点マップデータは4000くらいでやる。
	//for (int i = 6; i < *mapData3D_MAX - 40; i += 40) {//*mapData3D_MAX はmapData.h
	//	for (int j = 0; j < 40; j++) {
	//		p_mapData3D[i + j].x = -380 + 20 * j;
	//		p_mapData3D[i + j].y = (i-6)/2  - 220;
	//		p_mapData3D[i + j].z = 0;
	//		if (i > *mapData3D_MAX / 3.1) {
	//			//std::cout <<"y座標"<< p_mapData3D[i + j].y << std::endl;
	//			//std::cin >> i;
	//			p_mapData3D[i + j].z = tan( 0* 3.1415926 / 180)*(i - (*mapData3D_MAX / 3.1) - 6) / 2;
	//		}
	//	}
	//}


	////地形と脛の干渉チェック用の座標群。(斜面への乗り移り用だから8点)
	//int i = *mapData3D_MAX - 40;//i=3960
	//p_mapData3D[i].x = -400;
	//p_mapData3D[i].y = -250;
	//p_mapData3D[i].z = 0;
	//++i;//3961
	//p_mapData3D[i].x = 420;
	//p_mapData3D[i].y = -250;
	//p_mapData3D[i].z = 0;
	//++i;//3962
	//p_mapData3D[i].x = 420;
	//p_mapData3D[i].y = 420;
	//p_mapData3D[i].z = 0;
	//++i;//3963
	//p_mapData3D[i].x = -400;
	//p_mapData3D[i].y = 420;
	//p_mapData3D[i].z = 0;
	////ここまでが水平面の4点


	//++i;//3964
	//p_mapData3D[i].x = -400;
	//p_mapData3D[i].y = 420;
	//p_mapData3D[i].z = 0;
	//++i;//3965
	//p_mapData3D[i].x = 420;
	//p_mapData3D[i].y = 420;
	//p_mapData3D[i].z = 0;
	//++i;//3966
	//p_mapData3D[i].x = 420;
	//p_mapData3D[i].y = p_mapData3D[*mapData3D_MAX - 80].y;
	//p_mapData3D[i].z = p_mapData3D[*mapData3D_MAX - 80].z;
	//++i;//3967
	//p_mapData3D[i].x = -400;
	//p_mapData3D[i].y = p_mapData3D[*mapData3D_MAX - 80].y;
	//p_mapData3D[i].z = p_mapData3D[*mapData3D_MAX - 80].z;
	////ここまでが斜面の4点


	//++i;//3968
	//p_mapData3D[i].x = -400;
	//p_mapData3D[i].y = 420;
	//p_mapData3D[i].z = 0;
	//++i;//3969
	//p_mapData3D[i].x = 420;
	//p_mapData3D[i].y = 420;
	//p_mapData3D[i].z = 0;
	//++i;//3970
	//p_mapData3D[i].x = 420;
	//p_mapData3D[i].y = p_mapData3D[*mapData3D_MAX - 80].y;
	//p_mapData3D[i].z = p_mapData3D[*mapData3D_MAX - 80].z;
	//++i;//3971
	//p_mapData3D[i].x = -400;
	//p_mapData3D[i].y = p_mapData3D[*mapData3D_MAX - 80].y;
	//p_mapData3D[i].z = p_mapData3D[*mapData3D_MAX - 80].z;
	//段差との数合わせi=3960~3971（余ったのデータ、斜面の4点と現状一緒。）


	//これは10mm間隔の格子点マップデータは16000ぐらいでやる　
	//for (int i = 6; i < *mapData3D_MAX - 80; i += 80) {//*mapData3D_MAX はmapData.h
	//	for (int j = 0; j < 80; j++) {
	//		p_mapData3D[i + j].x = -380 + 10 * j;
	//		p_mapData3D[i + j].y = (i - 6) / 8 - 220;
	//		p_mapData3D[i + j].z = 0;
	//		if (i > *mapData3D_MAX / 4) {
	//			p_mapData3D[i + j].z = -tan(15 * 3.1415926 / 180)*(i - (*mapData3D_MAX / 4) - 6) / 8;
	//		}
	//	}
	//}

	//⑤平面から高段差（±○mm）への昇降）
//for (int i = 6; i < *mapData3D_MAX - 40; i += 40) {//*mapData3D_MAX = 4000
//	for (int j = 0; j < 40; j++) {
//		p_mapData3D[i + j].x = -380 + 20 * j;
//		p_mapData3D[i + j].y = i / 2 - 6 - 220;
//		//	std::cout << p_mapData3D[i + j].x<< std::endl;
//		p_mapData3D[i + j].z = 0;
//
//		if (i > *mapData3D_MAX * 1/2) {//段差の奥行きの調整は分数
//			//if (i < 1200) {
//			//	p_mapData3D[i + j].y = 2000;
//			//}
//			//p_mapData3D[i + j].y = i-6 - 220;
//			p_mapData3D[i + j].z = 0;//ここを定数で変更すればその高さのギャップが生まれる
//		}
//		//if (i > *mapData3D_MAX * 2/3) {
//		//	//if (i < 1200) {
//		//	//	p_mapData3D[i + j].y = 2000;
//		//	//}
//		//	p_mapData3D[i + j].z = 150;//ここを定数で変更すればその高さのギャップが生まれる
//		//}
//
//	}
//}
//十分割
	//for (int i = 6; i < *mapData3D_MAX - 40; i += 40) {//*mapData3D_MAX = 4000
	//	for (int j = 0; j < 40; j++) {
	//		p_mapData3D[i + j].x = -380 + 20 * j;
	//		p_mapData3D[i + j].y = i/2 -6  - 220;
	//	//	std::cout << p_mapData3D[i + j].x<< std::endl;
	//		p_mapData3D[i + j].z = 150;

	//		if (i > *mapData3D_MAX* 3 / 10) {//段差の奥行きの調整は分数
	//			//if (i < 1200) {
	//			//	p_mapData3D[i + j].y = 2000;
	//			//}
	//			//p_mapData3D[i + j].y = i-6 - 220;
	//			p_mapData3D[i + j].z = 150;//ここを定数で変更すればその高さのギャップが生まれる
	//		}
	//		if (i > *mapData3D_MAX* 4 / 10) {
	//			//if (i < 1200) {
	//			//	p_mapData3D[i + j].y = 2000;
	//			//}
	//			p_mapData3D[i + j].z = 150-40;//ここを定数で変更すればその高さのギャップが生まれる
	//		}
	//		if (i > *mapData3D_MAX * 5/ 10) {
	//			//if (i < 1200) {
	//			//	p_mapData3D[i + j].y = 2000;
	//			//}
	//			p_mapData3D[i + j].z = 150-80;//ここを定数で変更すればその高さのギャップが生まれる
	//		}
	//		if (i > *mapData3D_MAX * 6 / 10) {
	//			//if (i < 1200) {
	//			//	p_mapData3D[i + j].y = 2000;
	//			//}
	//			p_mapData3D[i + j].z = 150-40;//ここを定数で変更すればその高さのギャップが生まれる
	//		}
	//		if (i > *mapData3D_MAX * 7 / 10) {
	//			//if (i < 1200) {
	//			//	p_mapData3D[i + j].y = 2000;
	//			//}
	//			p_mapData3D[i + j].z = 150;//ここを定数で変更すればその高さのギャップが生まれる
	//		}

	//		if (i > *mapData3D_MAX * 8 / 10) {
	//			//if (i < 1200) {
	//			//	p_mapData3D[i + j].y = 2000;
	//			//}
	//			p_mapData3D[i + j].z = 150;//ここを定数で変更すればその高さのギャップが生まれる
	//		}
	//	}
	//}



	//	//地形と脛の干渉チェック用の座標群。(2平面と鉛直面で12点)
	//	//⑤の段差の境界が1/2の場所のときのデータセット。
	//int i = *mapData3D_MAX - 40;//i=3960
	//p_mapData3D[i].x = -400;
	//p_mapData3D[i].y = -250;
	//p_mapData3D[i].z = 0;
	//++i;//3961
	//p_mapData3D[i].x = 420;
	//p_mapData3D[i].y = -250;
	//p_mapData3D[i].z = 0;
	//++i;//3962
	//p_mapData3D[i].x = 420;
	//p_mapData3D[i].y = 770;
	//p_mapData3D[i].z = 0;
	//++i;//3963
	//p_mapData3D[i].x = -400;
	//p_mapData3D[i].y = 770;
	//p_mapData3D[i].z = 0;
	////ここまでが水平面の4点

	//++i;//3964
	//p_mapData3D[i].x = -400;
	//p_mapData3D[i].y = 770;
	//p_mapData3D[i].z = 0;
	//++i;//3965
	//p_mapData3D[i].x = 420;
	//p_mapData3D[i].y = 770;
	//p_mapData3D[i].z = 0;
	//++i;//3966
	//p_mapData3D[i].x = 420;
	//p_mapData3D[i].y = 770;
	//p_mapData3D[i].z = p_mapData3D[*mapData3D_MAX - 80].z; //90;// p_mapData3D[*mapData3D_MAX - 80].z;
	//++i;//3967
	//p_mapData3D[i].x = -400;
	//p_mapData3D[i].y = 770;
	//p_mapData3D[i].z = p_mapData3D[*mapData3D_MAX - 80].z; //90;// p_mapData3D[*mapData3D_MAX - 80].z;

	////ここまでが鉛直面の4点
	//++i;//3968
	//p_mapData3D[i].x = -400;
	//p_mapData3D[i].y = 770;//830;//770;
	//p_mapData3D[i].z = p_mapData3D[*mapData3D_MAX - 80].z;//0;// p_mapData3D[*mapData3D_MAX - 80].z;
	//++i;//3969
	//p_mapData3D[i].x = 420;
	//p_mapData3D[i].y = 770;//830;// 770;
	//p_mapData3D[i].z = p_mapData3D[*mapData3D_MAX - 80].z;// 0;// p_mapData3D[*mapData3D_MAX - 80].z;
	//++i;//3970
	//p_mapData3D[i].x = 420;
	//p_mapData3D[i].y = p_mapData3D[*mapData3D_MAX - 80].y;//830;//p_mapData3D[*mapData3D_MAX - 80].y;
	//p_mapData3D[i].z = p_mapData3D[*mapData3D_MAX - 80].z; //90;// p_mapData3D[*mapData3D_MAX - 80].z;
	//++i;//39671
	//p_mapData3D[i].x = -400;
	//p_mapData3D[i].y = p_mapData3D[*mapData3D_MAX - 80].y; //830;//p_mapData3D[*mapData3D_MAX - 80].y;
	//p_mapData3D[i].z = p_mapData3D[*mapData3D_MAX - 80].z;//90;// p_mapData3D[*mapData3D_MAX - 80].z;
	////ここまでが段差の4点

	//⑥片側段差
	//for (int i = 6; i < *mapData3D_MAX - 20; i += 20) {//*mapData3D_MAX = 2000
	//	for (int j = 0; j < 20; j++) {
	//		p_mapData3D[i + j].x = -380 + 40 * j;
	//		p_mapData3D[i + j].y = i * 2- 220;
	//		p_mapData3D[i + j].z = 0;
	//		if (i > *mapData3D_MAX / 2 - 200) {
	//			if (j < 10) {
	//				p_mapData3D[i + j].z = 80;//ここを定数で変更すればその高さのギャップが生まれる
	//			}
	//			else {
	//				p_mapData3D[i + j].z = 0;//ここを定数で変更すればその高さのギャップが生まれる
	//			}
	//		}
	//	}
	//}


	int maxX = MAP_X_MIN;
	int maxY = MAP_Y_MIN;
	int x_count = 0;
	int y_count = 0;
	int rough_terrain_count = 0;
	int invalid_count = 0;
#ifdef SLOPE
	//勾配ベクトルを参照 z = f(x,y) = x *tan_roll + y * tan_pitch;
	double tan_pitch = sqrt(tan(THETA_SLOPE * Define::MY_PI / 180) * tan(THETA_SLOPE * Define::MY_PI / 180) / (1 + tan(XI_SLOPE * Define::MY_PI / 180)*tan(XI_SLOPE * Define::MY_PI / 180)));
	double tan_roll = tan(XI_SLOPE * Define::MY_PI / 180) * tan_pitch;
	//double z_sub = maxX * tan_roll + maxY * tan_pitch
#endif

#ifdef ISOSELES_TRIANGLE
	bool plus = false;
#endif	

	for (int i = 6; i < Define::MAPDATA3D_MAX; ++i) {
		maxX += FOOT_HOLD_XY_DIST;
		x_count++;
		if (maxX >= MAP_X_MAX - FOOT_HOLD_XY_DIST) {//yの最小値でxを最小値から最大値まで横に並べていって最大値まで達したら，yの最小値を更新して，ｘをまた最小値から横に並べる．
			maxX = MAP_X_MIN;
			maxY += FOOT_HOLD_XY_DIST;
			y_count++;

#ifdef ISOSELES_TRIANGLE
			if ((maxY - START_ROUGH_TARRAIN_Y) / DEPTH_TRI % 2 == 0 ) plus = false;
			else plus = true;
#endif

			x_count = 0;
		}

		if (maxY <= MAP_Y_MAX - FOOT_HOLD_XY_DIST) {//yがマップの最大値を超すまで脚設置可能点として３次元座標を保存する、この時点ではすべて平面
			p_mapData3D[i].x = maxX;
			p_mapData3D[i].y = maxY;
			p_mapData3D[i].z = 0;//この時点ではすべて水平面

			//↓ここから，不整地の領域に対してz(高さ)を変えて不整地に変更する．
			if (p_mapData3D[i].y > START_ROUGH_TARRAIN_Y) {
				++rough_terrain_count;
#ifdef SLOPE 
				p_mapData3D[i].z = -maxX * tan_roll - (maxY - START_ROUGH_TARRAIN_Y) * tan_pitch;
#endif
#ifdef STEP
				//std::cout << (maxY - START_ROUGH_TARRAIN_Y) % DEPTH_STEP << std::endl;
				p_mapData3D[i].z = ((maxY - START_ROUGH_TARRAIN_Y) / DEPTH_STEP +1) * HEIGHT_STEP;
#endif


#ifdef ISOSELES_TRIANGLE
				if (x_count % (WIDE_TRI / FOOT_HOLD_XY_DIST) == 0 && plus == true) {
					p_mapData3D[i].z = WIDE_TRI * tan(THETA_TRI*Define::MY_PI / 180);
					plus = false; }
				else if (x_count % (WIDE_TRI / FOOT_HOLD_XY_DIST) == 0 && plus == false) {
					p_mapData3D[i].z = WIDE_TRI * tan(THETA_TRI*Define::MY_PI / 180);
					p_mapData3D[i].z = 0;
					plus = true;
				}
				else if (x_count % (WIDE_TRI / FOOT_HOLD_XY_DIST) != 0 && plus == true) {
					p_mapData3D[i].z = FOOT_HOLD_XY_DIST * (x_count % (WIDE_TRI / FOOT_HOLD_XY_DIST)) * tan(THETA_TRI*Define::MY_PI / 180);
				}
				else if (x_count % (WIDE_TRI / FOOT_HOLD_XY_DIST) != 0 && plus == false) {
					//std::cout << "minus";
					p_mapData3D[i].z = WIDE_TRI * tan(THETA_TRI*Define::MY_PI / 180) - FOOT_HOLD_XY_DIST * (x_count % (WIDE_TRI / FOOT_HOLD_XY_DIST)) * tan(THETA_TRI*Define::MY_PI / 180);

				}
#endif
				//p_mapData3D[i].z = 130;

				//p_mapData3D[i].z = -40 - x_count * 2 + 2 * y_count;
			}

				
		}

		
	}

	//ここから，特定の領域をホールとする処理．ランダム，縦じま，横じま，斜めじまなどを用意	
	//幾何学模様
#ifndef HOLE_RANDOM

	maxX = MAP_X_MIN;
	maxY = MAP_Y_MIN;
	x_count = 0;
	y_count = 0;
	invalid_count = 0;
	for (int i = 6; i < MAPDATA3D_MAX; ++i) {
		maxX += FOOT_HOLD_XY_DIST;
		x_count++;
		if (maxX >= MAP_X_MAX - FOOT_HOLD_XY_DIST) {//yの最小値でxを最小値から最大値まで横に並べていって最大値まで達したら，yの最小値を更新して，ｘをまた最小値から横に並べる．
			maxX = MAP_X_MIN;
			maxY += FOOT_HOLD_XY_DIST;
			y_count++;
			x_count = 0;
		}
		//std::cout << ((maxX - MAP_X_MIN) / SQUARE_SIZE) << std::endl;
		//std::cout << maxY << std::endl;
		if (maxY <= MAP_Y_MAX - FOOT_HOLD_XY_DIST) {//yがマップの最大値を超すまで脚設置可能点として３次元座標を保存する、この時点ではすべて平面
			if (p_mapData3D[i].y > START_ROUGH_TARRAIN_Y) {
				if (((maxX - MAP_X_MIN) / SQUARE_SIZE) % 2 != 0) {	//縦じま
				//if (((maxY - MAP_Y_MIN) / SQUARE_SIZE) % 2 != 0) {	//横じま
				//if (((maxY - MAP_Y_MIN) / SQUARE_SIZE + (maxX - MAP_X_MIN) / SQUARE_SIZE) % 2 != 0) { //斜めじま
				//if (((maxY - MAP_Y_MIN) / SQUARE_SIZE) % 2 != 0&& ((maxX - MAP_X_MIN) / SQUARE_SIZE) % 2 != 0) { //網目
				//if (((maxY - MAP_Y_MIN) / SQUARE_SIZE) % 2 != 0 || ((maxX - MAP_X_MIN) / SQUARE_SIZE) % 2 != 0) {	//格子点（網目の逆）
					p_mapData3D[i].x = INVALID_FOOT_HOLD;
					p_mapData3D[i].y = INVALID_FOOT_HOLD;
					p_mapData3D[i].z = INVALID_FOOT_HOLD;
					++invalid_count;
				}
			}
		}
	}

#endif

	//ランダム足場にする(もともと整地の座標が格納されている前提で使用)
#ifdef HOLE_RANDOM
		int Random1[Define::MAPDATA3D_MAX];//乱数を格納する配列1//足場の数を入れる要は*mapData3D_MAXの値
		int Random2[Define::MAPDATA3D_MAX];
		int Random3[Define::MAPDATA3D_MAX];
		GetRandom(*mapData3D_MAX, 1, 100, Random1);//1~100の乱数を格納//割合%を表す
		GetRandom(*mapData3D_MAX, 0, HEIGHT_MAGNIFICATION, Random2);//乱数を格納//とりあえずランダムなずれの倍率を表す
		GetRandom(*mapData3D_MAX, 0, Define::MAPDATA3D_MAX - 1, Random3);//乱数を格納//ランダムなインデックス番号//1点ずつ

		for (int i = 6; i < Define::MAPDATA3D_MAX; ++i) {
			for (int xx = MAP_X_MIN; xx < MAP_X_MAX; xx += SQUARE_SIZE) {
				for (int yy = START_ROUGH_TARRAIN_Y; yy < MAP_Y_MAX; yy += SQUARE_SIZE) {
					if (xx <= p_mapData3D[i].x && p_mapData3D[i].x < xx + SQUARE_SIZE) {
						if (yy <= p_mapData3D[i].y && p_mapData3D[i].y < yy + SQUARE_SIZE) {
							if (xx < 0) xx += 1050;
							if (yy < 0) yy += 1050;
							if (Random1[Random3[20 * xx / SQUARE_SIZE + yy / SQUARE_SIZE]] < HOLE_RATE) {//足場を無くす割合(%)
								p_mapData3D[i].x = INVALID_FOOT_HOLD;//絶対に脚が届かないところに足場を飛ばす
								p_mapData3D[i].y = INVALID_FOOT_HOLD;//絶対に脚が届かないところに足場を飛ばす
								p_mapData3D[i].z = INVALID_FOOT_HOLD;//絶対に脚が届かないところに足場を飛ばす
							}
#ifdef RANDOM_ADD_Z //高さ方向にも乱数を振りたいときはヘッダーファイルでdefine
							else if (Random1[20 * xx / SQUARE_SIZE + yy / SQUARE_SIZE] % 2 == 0) {
								p_mapData3D[i].z += Random2[Random3[20 * xx / SQUARE_SIZE + yy / SQUARE_SIZE]] * RANDOM_ADD_Z;
							}
							else {
								p_mapData3D[i].z -= Random2[Random3[20 * xx / SQUARE_SIZE + yy / SQUARE_SIZE]] * RANDOM_ADD_Z;
							}
#endif
						}
					}
				}
			}

		}
#endif

	//脚設置可能点のデータをexcelファイルに出力する
		if (f == 0) {
			std::cout << "ホール率 = " << invalid_count * 100 / rough_terrain_count << "%" << std::endl;
			//std::cin >> go;
			std::string write_mapdata_file_name = "map.csv";//作成されたexcelファイル名は適宜地形条件に応じたファイル名に変更して，mapdatasetフォルダにまとめること20210111hato
			std::ofstream write_mapdata_file;
			write_mapdata_file.open(write_mapdata_file_name);
			WriteMapDataToFile(write_mapdata_file, p_mapData3D, mapData3D_MAX);
		}

#ifdef READMAPDATA_FROM_CSV
	////ファイルから接地点読み取る用
	std::string file = "map.csv"; //○○.csvの○○は読み込みたいファイル名に変更すること20210111hato
	int mapff = ReadMapDataFromFile(file, p_mapData3D);
	p_mapData3D[0] = phantomX.getGlobalLegPos(0);
	p_mapData3D[1] = phantomX.getGlobalLegPos(1);
	p_mapData3D[2] = phantomX.getGlobalLegPos(2);
	p_mapData3D[3] = phantomX.getGlobalLegPos(3);
	p_mapData3D[4] = phantomX.getGlobalLegPos(4);
	p_mapData3D[5] = phantomX.getGlobalLegPos(5);
#endif
	////横縞
	//const double HHH = 0; // 初期変位
	//const double degree = 15; // 何度傾斜させるか
	//int kyori =100;//距離 +40が間隔
	//int sima = 3;//何行ごとに間が空くか
	//for(int i = 6; i < *mapData3D_MAX - 20; i+=20){
	//	for(int j = 0; j < 20; j++){
	//		p_mapData3D[i+j].x = -380 + 40 * j;
	//		p_mapData3D[i+j].y = i/20 * 100 -300;//kotti
	//		p_mapData3D[i + j].z = 30;
	//		if(sima <= 0){
	//		}else if(i / 20 / sima){
	//			p_mapData3D[i+j+6].y += i / 20 / sima * kyori;
	//		}
	//	}
	//}

	//片側の段差のやつ
	//int mn = 102;
	//for (int i = 6; i < mn; ++i) {
	//	if ((i - 6) % 8 < 4) {
	//		p_mapData3D[i].x = 50 * ((i - 6) % 4) - 250;
	//	} else if ((i - 6) % 8 >= 4) {
	//		p_mapData3D[i].x = 50 * ((i - 6) % 4) + 100;
	//	}
	//	p_mapData3D[i].y = ((i - 6) / 8) * 50 - 200;
	//	p_mapData3D[i].z = 0;
	//}

	//double offset = 50;
	//double s = 400;
	//int sima = 2;
	//double kyori = 50;
	//int dannn = 64;
	//for (int i = mn; i < mn + dannn; ++i) {

	//	if ((i - mn) % 8 < 4) {
	//		p_mapData3D[i].x = 50 * ((i - 6) % 4) - 250;
	//	} else if ((i - mn) % 8 >= 4) {
	//		p_mapData3D[i].x = 50 * ((i - 6) % 4) + 100;
	//	}

	//	p_mapData3D[i].y = ((i - mn) / 8) * 50 + s + offset;
	//	if (sima <= 0) {
	//	} else if ((i - mn) / 8 / sima) {
	//		p_mapData3D[i].y += (i - mn) / 8 / sima * kyori;
	//	}

	//	if (p_mapData3D[i].y < s || p_mapData3D[i].y > 600 + s) {
	//		p_mapData3D[i].z = 0;
	//	} else if ((p_mapData3D[i].y >= s && p_mapData3D[i].y < s + 150) || (p_mapData3D[i].y > s + 450 && p_mapData3D[i].y <= s + 600)) {
	//		if (p_mapData3D[i].x >= 0 && p_mapData3D[i].x < 300) {
	//			p_mapData3D[i].z = 40;
	//		} else {
	//			p_mapData3D[i].z = 0;
	//		}
	//	} else if ((p_mapData3D[i].y >= s + 150 && p_mapData3D[i].y <= s + 450)) {
	//		if (p_mapData3D[i].x >= 0 && p_mapData3D[i].x < 300) {
	//			p_mapData3D[i].z = 80;
	//		} else {
	//			p_mapData3D[i].z = 0;
	//		}
	//	} else {
	//		p_mapData3D[i].z = 0;
	//	}
	//}

	//for (int i = mn + dannn; i < mn + dannn + 120; ++i) {
	//	if ((i - 6) % 8 < 4) {
	//		p_mapData3D[i].x = 50 * ((i - mn + dannn) % 4) - 250;
	//	} else if ((i - mn + dannn) % 8 >= 4) {
	//		p_mapData3D[i].x = 50 * ((i - mn + dannn) % 4) + 100;
	//	}
	//	p_mapData3D[i].y = ((i - mn + dannn) / 8) * 50 + 250;
	//	p_mapData3D[i].z = 0;
	//}

//階段状のやつ
//const double HHm = -20; // 初期変位	ロボットから見てz方向可動範囲130mm中心とするとでロボットの胴体高さ110だからグローバル座標は-20
//const double degree = 19.6; // 何度傾斜させるか
//const double lamda = 225.0;
//const double b = 112.5;	//段差の幅
//int kyori = 10;//距離
//int sima = 1;//何行ごとに間が空くか
//for (int i = 0; i < *mapData3D_MAX - 26; i += 20) {
//	for (int j = 0; j < 20; j++) {
//		p_mapData3D[i + j + 6].x = -380 + 40 * j;
//		p_mapData3D[i + j + 6].y = i * 4 - 2 * lamda;
//		if (sima <= 0) {
//		}
//		else if (i / 20 / sima) {
//			p_mapData3D[i + j + 6].y += i / 20 / sima * kyori;
//		}
//
//		int f = int((p_mapData3D[i + j + 6].y + lamda / 2) / lamda);
//		if (f > 0) {
//			if ((f % 2) == 0) {
//				double h = fmod(p_mapData3D[i + j + 6].y + lamda / 2, lamda) - lamda / 2;	//-450~450に直す
//				int ma = int(h / (b / 2));
//				if (ma % 2 == 0) {
//					p_mapData3D[i + j + 6].z = HHm + ma / 2 * b * sin(degree * Define::MY_PI / 180.0);
//				}
//				else {
//					p_mapData3D[i + j + 6].z = HHm + (ma - int(ma / 2)) * b * sin(degree * Define::MY_PI / 180.0);
//				}
//			}
//			else {
//				double h = fmod(p_mapData3D[i + j + 6].y + lamda / 2, lamda) - lamda / 2;	//-450~450に直す
//				int ma = int(h / (b / 2));
//				if (ma % 2 == 0) {
//					p_mapData3D[i + j + 6].z = HHm - ma / 2 * b * sin(degree * Define::MY_PI / 180.0);
//				}
//				else {
//					p_mapData3D[i + j + 6].z = HHm - (ma - int(ma / 2)) * b * sin(degree * Define::MY_PI / 180.0);
//				}
//			}
//		}
//		else if (f < 0) {
//			if ((f % 2) == 0) {
//				double h = fmod(p_mapData3D[i + j + 6].y + lamda / 2, lamda) + lamda / 2;	//-450~450に直す
//				int ma = int(h / (b / 2));
//				if (ma % 2 == 0) {
//					p_mapData3D[i + j + 6].z = HHm - ma / 2 * b * sin(degree * Define::MY_PI / 180.0);
//				}
//				else {
//					p_mapData3D[i + j + 6].z = HHm - (ma - int(ma / 2)) * b * sin(degree * Define::MY_PI / 180.0);
//				}
//			}
//			else {
//				double h = fmod(p_mapData3D[i + j + 6].y + lamda / 2, lamda) + lamda / 2;	//-450~450に直す
//				int ma = int(h / (b / 2));
//				if (ma % 2 == 0) {
//					p_mapData3D[i + j + 6].z = HHm + ma / 2 * b * sin(degree * Define::MY_PI / 180.0);
//				}
//				else {
//					p_mapData3D[i + j + 6].z = HHm + (ma - int(ma / 2)) * b * sin(degree * Define::MY_PI / 180.0);
//				}
//			}
//		}
//		else if (f == 0) {
//			if ((p_mapData3D[i + j + 6].y + lamda / 2) > 0) {
//				double h = fmod(p_mapData3D[i + j + 6].y + lamda / 2, lamda) - lamda / 2;
//				int ma = int(h / (b / 2));
//				if (ma % 2 == 0) {
//					p_mapData3D[i + j + 6].z = HHm + ma / 2 * b * sin(degree * Define::MY_PI / 180.0);
//				}
//				else {
//					p_mapData3D[i + j + 6].z = HHm + (ma - int(ma / 2)) * b * sin(degree * Define::MY_PI / 180.0);
//				}
//			}
//			else {
//				double h = fmod(p_mapData3D[i + j + 6].y + lamda / 2, lamda) + lamda / 2;	//-450~450に直す
//				int ma = int(h / (b / 2));
//				if (ma % 2 == 0) {
//					p_mapData3D[i + j + 6].z = HHm - ma / 2 * b * sin(degree * Define::MY_PI / 180.0);
//				}
//				else {
//					p_mapData3D[i + j + 6].z = HHm - (ma - int(ma / 2)) * b * sin(degree * Define::MY_PI / 180.0);
//				}
//			}
//		}
//	}
//}


	//実機実験用190826
	//int mn = 78;
	//for (int i = 6; i < mn; ++i) {
	//	if ((i - 6) % 8 < 4) {
	//		p_mapData3D[i].x = 50 * ((i - 6) % 4) - 250;
	//	} else if ((i - 6) % 8 >= 4) {
	//		p_mapData3D[i].x = 50 * ((i - 6) % 4) + 100;
	//	}
	//	p_mapData3D[i].y = ((i - 6) / 8) * 50 - 200;
	//	p_mapData3D[i].z = 0;
	//}
	////-250 < x < 250
	////-200 < y < 1200
	//double offset = 25;
	//double s = 400;
	//int dannn = 64;
	//srand(1);	//乱数の初期値を変える
	//for (int i = mn; i < *mapData3D_MAX; ++i) {

	//	bool flag = true;
	//	myvector::VECTOR temp;
	//	while (flag) {
	//		flag = false;

	//		temp.x = ((int)rand() % 500) - 250;
	//		temp.y = ((int)rand() % 1300) + 250;
	//		temp.z = 0;

	//		if (temp.y > s - offset && temp.y < s + offset) {
	//			flag = true;
	//			continue;
	//		} else if (temp.y > s + 150 - offset && temp.y < s + 150 + offset) {
	//			flag = true;
	//			continue;
	//		} else if (temp.y > s + 450 - offset && temp.y < s + 450 + offset) {
	//			flag = true;
	//			continue;
	//		} else if (temp.y > s + 600 - offset && temp.y < s + 600 + offset) {
	//			flag = true;
	//			continue;
	//		}

	//		for (int j = 0; j < i; ++j) {
	//			if (myvector::V2Mag2(temp, p_mapData3D[j]) < 50) {
	//				flag = true;
	//				break;
	//			}

	//		}
	//	}

	//	p_mapData3D[i] = temp;

	//	if (p_mapData3D[i].y < s || p_mapData3D[i].y > 600 + s) {
	//		p_mapData3D[i].z = 0;
	//	} else if ((p_mapData3D[i].y >= s && p_mapData3D[i].y < s + 150) || (p_mapData3D[i].y > s + 450 && p_mapData3D[i].y <= s + 600)) {
	//		p_mapData3D[i].z = 40;
	//	} else if ((p_mapData3D[i].y >= s + 150 && p_mapData3D[i].y <= s + 450)) {
	//		p_mapData3D[i].z = 80;
	//	} else {
	//		p_mapData3D[i].z = 0;
	//	}
	//}

	////チェック模様?
	//for(int i = 6; i < *mapData3D_MAX - 10; i+=10){
	//	for(int j = 0; j < 10; j++){
	//		p_mapData3D[i+j].x = -380 + 80 * j;
	//		p_mapData3D[i+j].y = i * 4 - 380;
	//		p_mapData3D[i+j].z = 0;
	//		if(i % 20 < 10){
	//			p_mapData3D[i+j].x += 40;
	//		}
	//	}
	//}

	//////一定間隔ごとに並べる
	//int startX = -380;	//xの左端
	//int endX = 380;		//xの右端
	//int habaX = 120;	//xの幅
	//int startY = -380;	//yの下端
	//int habaY = 40;		//yの幅
	//for(int i = 6; i < *mapData3D_MAX; i++){
	//	p_mapData3D[i].x = startX + habaX * (i - 6) % (endX - startX);
	//	p_mapData3D[i].y = habaY * int(habaX * (i - 6) / (endX - startX)) + startY;
	//	p_mapData3D[i].z = 0;
	//}

	//円形の脚接地点
	//float th;	//角度
	//float dth;	//角度変化
	//float r =50.0;	//最小半径
	//float dr = 50.0;	//ひとつ外側の円との幅
	//int devide = 2;		//分割数
	//myvector::VECTOR center = {-0,0,0};	//中心
	//int num = 6;	//脚接地番号
	//float thstart = -0;
	//float thend = 359;

	//p_mapData3D[num++] = center;
	//while(num <  *mapData3D_MAX){
	//	for(th = thstart; th < thend; th += dth, num++){
	//		if(num >  *mapData3D_MAX) break;
	//		if(th == thstart){
	//			p_mapData3D[num].y = p_mapData3D[num].z = 0.0;
	//			p_mapData3D[num].x = center.x + r;
	//			r += dr;	//次は50mm外側の円
	//			//if(600 < r && r < 800) r = r + int(800 - r) / int(dr) * dr; //一定の半径の場所を抜く
	//			devide = 2;
	//			//dthの決定
	//			while(1){
	//				dth = 360.0 / (float)devide;
	//				p_mapData3D[0] = myvector::VRot(p_mapData3D[num], center, 0, 0, dth/180.0*Define::MY_PI);
	//				if(myvector::VMag(myvector::subVec(p_mapData3D[0],p_mapData3D[num])) > 50.0) devide++;
	//				else{
	//					devide--;
	//					p_mapData3D[num] = myvector::VRot(p_mapData3D[num], center, 0, 0, th/180.0*Define::MY_PI);
	//					break;
	//				}
	//			}
	//		}else{
	//			p_mapData3D[num] = myvector::VRot(p_mapData3D[num - 1], center, 0, 0, dth/180.0*Define::MY_PI);
	//			//一部の脚接地点を抜く
	//			if(th > 50 && th < 58 && 0 < r && r < 500){
	//				p_mapData3D[num] = myvector::VRot(p_mapData3D[num - 1], center, 0, 0, (5.0 * dth)/180.0*Define::MY_PI);//4つ連続で抜く
	//				th = th + 4.0 * dth; //一定の半径の場所を抜く
	//			}
	//		}
	//	}	
	//}

	////円形の脚接地点(放射状)
	//float th;	//角度
	//float r = 50.0;	//最小半径
	//int devide = 12;		//分割数
	//float dth = 360.0/float(devide);	//角度変化
	//myvector::VECTOR center = {-0,0,0};	//中心
	//int num = 6;	//脚接地番号
	//float thstart = -0;
	//float thend = 359;

	//p_mapData3D[num++] = center;
	//while(num <  *mapData3D_MAX){
	//	for(th = thstart; th < thend; th += dth, num++){
	//		if(num >  *mapData3D_MAX) break;
	//		if(th == thstart){
	//			p_mapData3D[num].y = p_mapData3D[num].z = 0.0;
	//			p_mapData3D[num].x = center.x + r;
	//			r += 50;	//次は50mm外側の円

	//			p_mapData3D[num] = myvector::VRot(p_mapData3D[num], center, 0, 0, th/180.0*Define::MY_PI);
	//		}else{
	//			p_mapData3D[num] = myvector::VRot(p_mapData3D[num - 1], center, 0, 0, dth/180.0*Define::MY_PI);
	//		}
	//	}	
//	}


	////○　　　○が続く(y方向)x=±250,230,210失敗,200途中からトライポット
	//for(int i = 6; i < *mapData3D_MAX-2; i+=2){

	//	p_mapData3D[i].x =- 200;
	//	p_mapData3D[i].y = i * 20 - 300;
	//	p_mapData3D[i].z = 0;

	//	p_mapData3D[i+1].x =200;
	//	p_mapData3D[i+1].y = i * 20 - 300;
	//	p_mapData3D[i+1].z = 0;
	//}

	////○　　　○が続く(x方向)y=200歩けない
	//for(int i = 6; i < *mapData3D_MAX-2; i+=2){
	//
	//	p_mapData3D[i].y =- 200;
	//	p_mapData3D[i].x = i * 20 - 300;
	//	p_mapData3D[i].z = 0;

	//	p_mapData3D[i+1].y =200;
	//	p_mapData3D[i+1].x = i * 20 - 300;
	//	p_mapData3D[i+1].z = 0;
	//}

	////○○○　　○○○が続く
	//for(int i = 6; i < *mapData3D_MAX-6; i+=6){
	//	for(int j = 0; j < 6; j++){
	//		p_mapData3D[i+j].x = -250 + 50 * j;
	//		p_mapData3D[i+j].y = i * 7 - 300;
	//		p_mapData3D[i+j].z = 0;
	//		if(j > 2){
	//			p_mapData3D[i+j].x = 50 * j;
	//		}
	//	}
	//}

	////崖状
	//for(int i = 6; i < *mapData3D_MAX-20; i+=20){
	//	for(int j = 0; j < 20; j++){
	//		p_mapData3D[i+j].x = -380 + i * 0.5 + 40 * j;
	//		p_mapData3D[i+j].y = i * 2 - 220;
	//		p_mapData3D[i+j].z = 0;
	//	}
	//}

//		for(int i = 6; i < *mapData3D_MAX-9; i+=9){
//			int henkou = 90;
//	//if(i > 90 && i < 109)continue;	
//		if((i+8)<henkou){
//		p_mapData3D[i].x =- 300;
//		p_mapData3D[i].y = i * 7 - 300;
//		p_mapData3D[i].z = 0;
//
//		p_mapData3D[i+1].x =- 225;
//		p_mapData3D[i+1].y = i * 7 - 300;
//		p_mapData3D[i+1].z = 0;
//
//		p_mapData3D[i+2].x =- 150;
//		p_mapData3D[i+2].y = i * 7 - 300;
//		p_mapData3D[i+2].z = 0;
//
//		p_mapData3D[i+3].x =- 75;
//		p_mapData3D[i+3].y = i * 7 - 300;
//		p_mapData3D[i+3].z = 0;
//
////	if(i > 90 && i < 112)continue;
//
//		p_mapData3D[i+4].x = 75;
//		p_mapData3D[i+4].y = i * 7 - 300;
//		p_mapData3D[i+4].z = 0;
//
//		p_mapData3D[i+5].x = 150;
//		p_mapData3D[i+5].y = i * 7 - 300;
//		p_mapData3D[i+5].z = 0;
//
//		p_mapData3D[i+6].x = 225;
//		p_mapData3D[i+6].y = i * 7 - 300;
//		p_mapData3D[i+6].z = 0;
//
//		p_mapData3D[i+7].x = 300;
//		p_mapData3D[i+7].y = i * 7 - 300;
//		p_mapData3D[i+7].z = 0;
//
//		p_mapData3D[i+8].x = 0;
//		p_mapData3D[i+8].y = i * 7 - 300;
//		p_mapData3D[i+8].z = 0;
//		}else if(i<300){
//		p_mapData3D[i].x =- 300 + 675;
//		p_mapData3D[i].y = i * 7 - 300 -300;
//		p_mapData3D[i].z = 0;
//
//		p_mapData3D[i+1].x =- 225 + 675;
//		p_mapData3D[i+1].y = i * 7 - 300 -300;
//		p_mapData3D[i+1].z = 0;
//
//		p_mapData3D[i+2].x =- 150 + 675;
//		p_mapData3D[i+2].y = i * 7 - 300 -300;
//		p_mapData3D[i+2].z = 0;
//
//		p_mapData3D[i+3].x =- 75 + 675;
//		p_mapData3D[i+3].y = i * 7 - 300 -300;
//		p_mapData3D[i+3].z = 0;
//
////	if(i > 90 && i < 112)continue;
//
//		p_mapData3D[i+4].x = 75 + 675;
//		p_mapData3D[i+4].y = i * 7 - 300 -300;
//		p_mapData3D[i+4].z = 0;
//
//		p_mapData3D[i+5].x = 150 + 675;
//		p_mapData3D[i+5].y = i * 7 - 300 -300;
//		p_mapData3D[i+5].z = 0;
//
//		p_mapData3D[i+6].x = 225 + 675;
//		p_mapData3D[i+6].y = i * 7 - 300 -300;
//		p_mapData3D[i+6].z = 0;
//
//		p_mapData3D[i+7].x = 300 + 675;
//		p_mapData3D[i+7].y = i * 7 - 300 -300;
//		p_mapData3D[i+7].z = 0;
//
//		p_mapData3D[i+8].x = 0 + 675;
//		p_mapData3D[i+8].y = i * 7 - 300 -300;
//		p_mapData3D[i+8].z = 0;
//		}else{
//		p_mapData3D[i].x =- 300;
//		p_mapData3D[i].y = i * 7 - 300 -100;
//		p_mapData3D[i].z = 0;
//
//		p_mapData3D[i+1].x =- 225;
//		p_mapData3D[i+1].y = i * 7 - 300 -100;
//		p_mapData3D[i+1].z = 0;
//
//		p_mapData3D[i+2].x =- 150;
//		p_mapData3D[i+2].y = i * 7 - 300 -100;
//		p_mapData3D[i+2].z = 0;
//
//		p_mapData3D[i+3].x =- 75;
//		p_mapData3D[i+3].y = i * 7 - 300 -100;
//		p_mapData3D[i+3].z = 0;
//
////	if(i > 90 && i < 112)continue;
//
//		p_mapData3D[i+4].x = 75;
//		p_mapData3D[i+4].y = i * 7 - 300;
//		p_mapData3D[i+4].z = 0;
//
//		p_mapData3D[i+5].x = 150;
//		p_mapData3D[i+5].y = i * 7 - 300;
//		p_mapData3D[i+5].z = 0;
//
//		p_mapData3D[i+6].x = 225;
//		p_mapData3D[i+6].y = i * 7 - 300;
//		p_mapData3D[i+6].z = 0;
//
//		p_mapData3D[i+7].x = 300;
//		p_mapData3D[i+7].y = i * 7 - 300;
//		p_mapData3D[i+7].z = 0;
//
//		p_mapData3D[i+8].x = 0;
//		p_mapData3D[i+8].y = i * 7 - 300;
//		p_mapData3D[i+8].z = 0;
//		}
//	}

	//for(int i = 6; i < *mapData3D_MAX; i++){
	//	p_mapData3D[i].x = 1.5*double(i)*cos(double(i));
	//	p_mapData3D[i].y = 1.5*double(i)*sin(double(i));
	//	p_mapData3D[i].z = 0;
	//}

	/*//円形に配置　
	srand(time(NULL));	//乱数の初期値を変える
	//円形にランダムに配置
	std::mt19937 mt;            // メルセンヌ・ツイスタの32ビット版
	std::random_device rnd;     // 非決定的な乱数生成器
	mt.seed( rnd() );
	int max = 1400;		//最大範囲の半径
	int min = 600;		//最小範囲の半径
	float minAngle = -180 * Define::MY_PI / 180.0;
	float maxAngle = 180 * Define::MY_PI / 180.0;
	myvector::VECTOR center = {-1000,0,0};	//円の中心
	int n = 7;
	double x,y;
	p_mapData3D[6] = center;
	while(n < *mapData3D_MAX){
		x = ((int)mt() - (1 << 31) ) % (max * 2) - max;
		y = ((int)mt() - (1 << 31) ) % (max * 2) - max;
		if(atan2(y,x) < minAngle || atan2(y,x) > maxAngle) continue;//角度で制限
		if(sqrt(x*x + y*y) >= min && sqrt(x*x + y*y) <= max){
			p_mapData3D[n].x = x + center.x;				//脚着地可能点
			p_mapData3D[n].y = y + center.y;				//脚着地可能点
			p_mapData3D[n].z = 0;
			for(int i = 0; i < n; i++){//近い脚接地点は使用しない
				if(myvector::V2Mag2(p_mapData3D[n],p_mapData3D[i])<50){ //元70
					break;
				}else if(n == (i + 1)){
					n++;
					break;
				}
			}
		}
	}*/

	/*srand(3);
	int max = 1100;		//最大範囲の半径
	int min = 400;		//最小範囲の半径
	float minAngle = -180.0 * Define::MY_PI / 180.0;
	float maxAngle = -15.0 * Define::MY_PI / 180.0;
	myvector::VECTOR center = {-700,0,0};	//円の中心
	int n = mapff;
	double x,y;
	//p_mapData3D[6] = center;
	while(n <280){
		x = rand() % (max * 2) - max;
		y = rand() % (max * 2) - max;
		if(atan2(y,x) < minAngle || atan2(y,x) > maxAngle) continue;//角度で制限
		if(sqrt(x*x + y*y) >= min && sqrt(x*x + y*y) <= max){
			p_mapData3D[n].x = x + center.x;				//脚着地可能点
			p_mapData3D[n].y = y + center.y;				//脚着地可能点
			p_mapData3D[n].z = 0;
			for(int i = 0; i < n; i++){//近い脚接地点は使用しない
				if(myvector::V2Mag2(p_mapData3D[n],p_mapData3D[i])<70){
					break;
				}else if(n == (i + 1)){
					n++;
					break;
				}
			}
			//n++;
		}
	}*/




	//for(int i = 6; i < *mapData3D_MAX; i++){
	//	//----------------------------ランダム-------------------------------------
	//	p_mapData3D[i].x = (rand()%1000) - 500;				//脚着地可能点
	//	p_mapData3D[i].y = (rand()%5000) - 500;				//脚着地可能点
	//	p_mapData3D[i].z = 0;
	//	//-------------------------------------------------------------------------
	//}
	//for(int i = 6; i < *mapData3D_MAX; i++){
	//	//----------------------------ランダム-------------------------------------
	//	p_mapData3D[i].x = ((rand()%30) - 15)*50;				//脚着地可能点
	//	p_mapData3D[i].y = ((rand()%30) - 15)*50;				//脚着地可能点
	//	p_mapData3D[i].z = 0;
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

	//if(i < *mapData3D_MAX){
	//	for(; i < *mapData3D_MAX; i++){
	//		p_mapData3D[i].x = 10000;
	//		p_mapData3D[i].y = 10000;
	//		p_mapData3D[i].z = 0;
	//	}
	//}
	//マップをロボットから見た方向,位置に変換
	//for(int i=6; i < *mapData3D_MAX; i++){
	//	p_mapData3D[i] = myvector::addVec(p_mapData3D[i],myvector::VGet(CurrentCondition->global_center_of_mass.x,CurrentCondition->global_center_of_mass.y,0.0));
	//	p_mapData3D[i] = myvector::VRot(p_mapData3D[i],myvector::VGet(CurrentCondition->global_center_of_mass.x,CurrentCondition->global_center_of_mass.y,0.0), -CurrentCondition->pitch, -CurrentCondition->roll, -CurrentCondition->yaw);
	//}



}


//マップの平行移動
void recalMap(myvector::SVector *p_mapData3D, int mapData3D_MAX, LNODE* CurrentCondition, LNODE* PastCondition)
{
	for(int i = 0; i < mapData3D_MAX; i ++)
	{
		//ひとつ前のロボット座標→グローバル座標→現在のロボット座標
		p_mapData3D[i] = myvector::VRot(p_mapData3D[i], PastCondition->global_center_of_mass, PastCondition->pitch, PastCondition->roll, PastCondition->yaw);
		p_mapData3D[i] = myvector::addVec(p_mapData3D[i], PastCondition->global_center_of_mass);
		p_mapData3D[i] = myvector::subVec(p_mapData3D[i], CurrentCondition->global_center_of_mass);
		p_mapData3D[i] = myvector::VRot(p_mapData3D[i], CurrentCondition->global_center_of_mass, -CurrentCondition->pitch, -CurrentCondition->roll, -CurrentCondition->yaw);
	}
}

void startMap(myvector::SVector *allmapData, myvector::SVector *mapData, int mapData3D_MAX){//ロボット周辺を既知の脚接地点とする
	for(int i = 0; i < mapData3D_MAX; i ++){
		if(myvector::V2Mag(allmapData[i]) < 300){
			mapData[i] = allmapData[i];
		}else{
			mapData[i] = myvector::VGet(10000,10000,0);
		}
	}
}

void DetectionPoint(myvector::SVector *allmapData, myvector::SVector *mapData, int mapData3D_MAX, LNODE* CurrentCondition){//扇形内の脚接地点を取得
	myvector::SVector center = myvector::VGet(CurrentCondition->global_center_of_mass.x,CurrentCondition->global_center_of_mass.y,0.0);
	myvector::SVector center2 = myvector::VGet(0.0, 0.0, 0.0);
	myvector::SVector temp[1000];
	float startAng = 45.0f / 180.0f * (float)Define::MY_PI + (float)CurrentCondition->yaw;
	float endAng = 135.0f / 180.0f * (float)Define::MY_PI + (float)CurrentCondition->yaw;
	float sx = cos(startAng);
	float sy = sin(startAng);
	float ex = cos(endAng);
	float ey = sin(endAng);
	for(int i = 0; i < mapData3D_MAX; i ++){
		temp[i] = myvector::VRot(allmapData[i],center, CurrentCondition->pitch, CurrentCondition->roll, CurrentCondition->yaw);
		temp[i] = myvector::addVec(temp[i],center);
		if(myvector::V2Mag2(temp[i],center) < 500){//とりあえず距離で分ける
			//角度で判断
			if(sx * (temp[i].y - center.y) - (temp[i].x - center.x) * sy > 0.001){
				if(ex * (temp[i].y - center.y) - (temp[i].x - center.x) * ey < -0.001){
					mapData[i] = allmapData[i];
				}
			}
		}
	}
}

int ReadMapDataFromFile(const std::string& filename, myvector::SVector *map, const char delimiter) 
{
	std::ifstream ifs(filename);//読み込むファイル
	std::string line;//ファイルの行
	int j = 0;

	if (!ifs.is_open()) {
		std::cout << "マップデータファイルが開けません．" << std::endl;
		while (1);
	}

	while (std::getline(ifs, line)) {//行ごとに読み込み
		std::istringstream stream(line);//
		std::string field;
		std::vector<std::string> result;

		while (std::getline(stream, field, delimiter)) {//行の読み込みが終わるまで
			result.push_back(field);//","ごとにresultにプッシュバック//この時点では文字として入力される
		}
		if (j != 0) {//8行目のマップデータから読み込む　j=0→1行目=変数名 j=1～6→2～7行目＝初期の脚先の座標が脚設置可能点になるから上書きしない
			map[j].x = stod(result.at(1));//stringをdoubleに変換して代入
			map[j].y = stod(result.at(2));
			map[j].z = stod(result.at(3));
		}
		//std::cout << map[j].x << "," << map[j].y << "," << map[j].z << std::endl;
		++j;//マップデータの数＝行の数
	}
	return j;//読み込んだマップデータの数を返す．

}

void WriteMapDataToFile(std::ofstream& filename, const myvector::SVector *map, const int* mapData3D_MAX) {
	//std::cout <<"*mapData3D_MAX = "<< *mapData3D_MAX << std::endl;
	//int stop;
	//std::cin >> stop;
	filename << "n" << "," << "x" << "," << "y" << "," << "z" << std::endl;
	for (int i = 0; i < *mapData3D_MAX; ++i) {
		filename << i << "," << map[i].x << "," << map[i].y << "," << map[i].z << "\n";
	}

}

void MapSqrtDivide(myvector::SVector *mapData, int mapDataNum, std::vector< std::vector< std::vector<myvector::SVector> > > &divideMapData, int pointNum[LP_DIVIDE_NUM][LP_DIVIDE_NUM]) {
	
	for (int i = 0; i < LP_DIVIDE_NUM; ++i) {
		for (int j = 0; j < LP_DIVIDE_NUM; ++j) {
			pointNum[i][j] = 0;	//ゼロクリア
		}
	}

	//脚接地可能点の存在している範囲 //引数にすべきか考え中
	double xMax = MAP_X_MAX;
	double xMin = MAP_X_MIN;
	double yMax = MAP_Y_MAX;
	double yMin = MAP_Y_MIN;

	double lengthX = (xMax - xMin) / LP_DIVIDE_NUM;	//1ブロックの長さ いまは２０
	double lengthY = (yMax - yMin) / LP_DIVIDE_NUM;//いまは100

	int x, y, t;
	for (int i = 0; i < mapDataNum; ++i) 
	{
		if (mapData[i].x == INVALID_FOOT_HOLD) continue;
		x = (int)((mapData[i].x - xMin) / lengthX);		//x方向のブロック番号
		y = (int)((mapData[i].y - yMin) / lengthY);		//y方向のブロック番号
		
		if (x >= LP_DIVIDE_NUM) x = LP_DIVIDE_NUM - 1;	//想定外の範囲の場合は端っこのブロックに収めるようにする
		if (x < 0) x = 0;
		if (y >= LP_DIVIDE_NUM) y = LP_DIVIDE_NUM - 1;
		if (y < 0) y = 0;

		t = pointNum[x][y];
		if (divideMapData[x][y].size() <= pointNum[x][y]) 
		{
			//足りなくなったら現在の倍サイズ確保　使いまわすときはswapで小さくする（resizeだとcapacityは変わらないため）
			divideMapData[x][y].resize(divideMapData[x][y].size() * 2);	
		}
		//divideMapData[x][y].push_back(mapData[i]);
		divideMapData[x][y][t] = mapData[i];
		++pointNum[x][y];
	}
}

void AreaDivide(myvector::SVector p1, myvector::SVector p2, int &x1, int &x2, int &y1, int &y2) 
{
	//与えられた座標からブロック番号を求める p1:四角形エリアの左下の点 p2:右上の点　　x1:最小　x2:最大　y1:最小　y2:最大
	double xMax = MAP_X_MAX;
	double xMin = MAP_X_MIN;
	double yMax = MAP_Y_MAX;
	double yMin = MAP_Y_MIN;

	double lengthX = (xMax - xMin) / LP_DIVIDE_NUM;	//1ブロックの長さ
	double lengthY = (yMax - yMin) / LP_DIVIDE_NUM;

	//ブロック番号計算
	x1 = (int)((p1.x - xMin) / lengthX);
	x2 = (int)((p2.x - xMin) / lengthX);
	y1 = (int)((p1.y - yMin) / lengthY);
	y2 = (int)((p2.y - yMin) / lengthY);
	
	if (x1 >= LP_DIVIDE_NUM) x1 = LP_DIVIDE_NUM - 1;	//想定外の範囲の場合は端っこのブロックに収めるようにする
	if (x1 < 0) x1 = 0;
	if (y1 >= LP_DIVIDE_NUM) y1 = LP_DIVIDE_NUM - 1;
	if (y1 < 0) y1 = 0;
	if (x2 >= LP_DIVIDE_NUM) x2 = LP_DIVIDE_NUM - 1;	//想定外の範囲の場合は端っこのブロックに収めるようにする
	if (x2 < 0) x2 = 0;
	if (y2 >= LP_DIVIDE_NUM) y2 = LP_DIVIDE_NUM - 1;
	if (y2 < 0) y2 = 0;
}

void ReadStartNodeFromFile(LNODE &node,const int f, const char delimiter) {
	//重心位置だけ変えればいい
	std::string name = "start_node_test";
	std::string num = std::to_string(f);
	std::string csv = ".csv";
	std::string readfilename;
	std::ifstream read_start_node;
	//std::ofstream pattern_log;

	readfilename = name + num + csv;
	read_start_node.open(readfilename);
	if (!read_start_node) {
		std::cout << "dont open gcom log file" << readfilename << std::endl;
		std::string stop;
		std::cin >> stop;
	}
	std::string line;//ファイルの行
	int i = 1;
	while (std::getline(read_start_node, line)) {//行ごとに読み込み
		std::istringstream stream(line);//
		std::string field;
		std::vector<std::string> result;

		while (std::getline(stream, field, delimiter)) {//行の読み込みが終わるまで
			result.push_back(field);//","ごとにresultにプッシュバック//この時点では文字として入力される
		}
		if (i == 1) {
			node.global_center_of_mass.x = stod(result.at(0));//stringをdoubleに変換して代入
			node.global_center_of_mass.y = stod(result.at(1));
			node.global_center_of_mass.z = stod(result.at(2));
		}
		else if (2 <= i && i <= 7) {
			node.Leg[i - 2].x = stod(result.at(0));
			node.Leg[i - 2].y = stod(result.at(1));
			node.Leg[i - 2].z = stod(result.at(2));
		}
		else if (8 <= i && i <= 13) {
			node.Leg2[i - 8].x = stod(result.at(0));
			node.Leg2[i - 8].y = stod(result.at(1));
			node.Leg2[i - 8].z = stod(result.at(2));
		}
		else if (i == 14) node.pitch = stod(result.at(0));
		else if (i == 15) node.roll = stod(result.at(0));
		else if (i == 16) node.yaw = stod(result.at(0));
		else if (i == 17) node.center_of_mass = stoi(result.at(0));
		else if (i == 18) node.leg_state = stoi(result.at(0));
		else if (i == 19) node.parent = NULL;
		else if (i == 20) node.node_height = stoi(result.at(0));
		else if (i == 21) node.debug = stoi(result.at(0));
		else if (i == 22) node.delta_comz = stod(result.at(0));
		++i;//マップデータの数＝行の数

	}
}

void WriteStartNodeToFile(const LNODE& node, const int f) 
{
	//重心位置だけ書き込めばいい
	std::string name = "start_node";
	std::string num = std::to_string(f);
	std::string csv = ".csv";
	std::string writefilename;
	std::ofstream write_start_node;
	//std::ofstream pattern_log;

	writefilename = name +num + csv;
	write_start_node.open(writefilename);
	if (!write_start_node) {
		std::cout << "dont open gcom log file" << writefilename << std::endl;
		std::string stop;
		std::cin >> stop;
	}

	write_start_node << node.global_center_of_mass.x << ", " << node.global_center_of_mass.y << ", " << node.global_center_of_mass.z << std::endl;

	write_start_node << node.Leg[0].x << ", " << node.Leg[0].y << ", " << node.Leg[0].z << std::endl;
	write_start_node << node.Leg[1].x << ", " << node.Leg[1].y << ", " << node.Leg[1].z << std::endl;
	write_start_node << node.Leg[2].x << ", " << node.Leg[2].y << ", " << node.Leg[2].z << std::endl;
	write_start_node << node.Leg[3].x << ", " << node.Leg[3].y << ", " << node.Leg[3].z << std::endl;
	write_start_node << node.Leg[4].x << ", " << node.Leg[4].y << ", " << node.Leg[4].z << std::endl;
	write_start_node << node.Leg[5].x << ", " << node.Leg[5].y << ", " << node.Leg[5].z << std::endl;
	write_start_node << node.Leg2[0].x << ", " << node.Leg2[0].y << ", " << node.Leg2[0].z << std::endl;
	write_start_node << node.Leg2[1].x << ", " << node.Leg2[1].y << ", " << node.Leg2[1].z << std::endl;
	write_start_node << node.Leg2[2].x << ", " << node.Leg2[2].y << ", " << node.Leg2[2].z << std::endl;
	write_start_node << node.Leg2[3].x << ", " << node.Leg2[3].y << ", " << node.Leg2[3].z << std::endl;
	write_start_node << node.Leg2[4].x << ", " << node.Leg2[4].y << ", " << node.Leg2[4].z << std::endl;
	write_start_node << node.Leg2[5].x << ", " << node.Leg2[5].y << ", " << node.Leg2[5].z << std::endl;
	write_start_node << node.pitch << std::endl;
	write_start_node << node.roll << std::endl;
	write_start_node << node.yaw << std::endl;
	write_start_node << node.center_of_mass << std::endl;
	write_start_node << node.leg_state << std::endl;
	write_start_node << node.parent << std::endl;
	write_start_node << node.node_height << std::endl;
	write_start_node << node.debug << std::endl;
	write_start_node << node.delta_comz << std::endl;

	write_start_node.close();
}

void SetConditionForStripe(LNODE &node,const int f) 
{
	double COM_Z = 130;
	int random_r[START_RANDOM_R];
	int random_theta[360];
	int random_l[1000];
	GetRandom(START_RANDOM_R, 0, START_RANDOM_R, random_r);
	GetRandom(360, 0, 360, random_theta);
	GetRandom(1000, 0, 1000, random_l);

	for (int i = 0; i < 1000; ++i) 
	{
		if (random_l[i] > 500)	random_l[i] -= 1000;
		//std::cout << random_l[i] << std::endl;
	}

	node.global_center_of_mass = myvector::VGet(random_l[1] + random_r[1] * cos(random_theta[1] * Define::MY_PI / 180), random_r[1] * sin(random_theta[1] * Define::MY_PI / 180), COM_Z);//重心位置グローバル
	//node.global_center_of_mass = myvector::VGet(random_l[500] + random_r[100]*cos(random_theta[180]*Define::MY_PI/180), random_r[100] * sin(random_theta[180]*Define::MY_PI/180),COM_Z);//重心位置グローバル
	//node.global_center_of_mass = myvector::VGet(0, 0, 160);//重心位置グローバル

	node.Leg[0] = myvector::VGet(120, 100, -COM_Z);//付け根から脚先の位置
	node.Leg[1] = myvector::VGet(130, 0, -COM_Z);
	node.Leg[2] = myvector::VGet(120, -100, -COM_Z);
	node.Leg[3] = myvector::VGet(-120, -100, -COM_Z);
	node.Leg[4] = myvector::VGet(-130, 0, -COM_Z);
	node.Leg[5] = myvector::VGet(-120, 100, -COM_Z);
	//脚の位置(z方向固定)
	node.Leg2[0] = myvector::VGet(120, 100, -COM_Z);
	node.Leg2[1] = myvector::VGet(130, 0, -COM_Z);
	node.Leg2[2] = myvector::VGet(120, -100, -COM_Z);
	node.Leg2[3] = myvector::VGet(-120, -100, -COM_Z);
	node.Leg2[4] = myvector::VGet(-130, 0, -COM_Z);
	node.Leg2[5] = myvector::VGet(-120, 100, -COM_Z);

	//姿勢テイトブライアン角グローバル
	node.pitch = 0.0;//x軸回転
	node.roll = 0.0;//y軸回転
	node.yaw = 0.0;//z軸回転
	node.center_of_mass = 0;//重心位置int
	node.leg_state = 0b00000000110011001100110011001100;
	node.parent = NULL;//親ノードのポインタ
	node.node_height = 1;//ノード高さ
	node.debug = 24;//現在運動履歴として使用,前回の脚上下ノード(上下運動をした場合)2桁,前回の動作1桁,前々回の動作1桁
	node.delta_comz = 0;

	//ReadStartNodeFromFile(node,f);
	//
	//WriteStartNodeToFile(node, f);
	//
	//node.global_center_of_mass = myvector::VGet(0,0, 130);//重心位置グローバル
	//
	////node.Leg[0] = myvector::VGet(120, 80, -110);//付け根から脚先の位置
	////node.Leg[1] = myvector::VGet(120, 0, -110);
	////node.Leg[2] = myvector::VGet(120, -80, -110);
	////node.Leg[3] = myvector::VGet(-120, -80, -110);
	////node.Leg[4] = myvector::VGet(-120, 0, -110);
	////node.Leg[5] = myvector::VGet(-120, 80, -110);
	//////脚の位置(z方向固定)
	////node.Leg2[0] = myvector::VGet(120, 80, -110);
	////node.Leg2[1] = myvector::VGet(120, 0, -110);
	////node.Leg2[2] = myvector::VGet(120, -80, -110);
	////node.Leg2[3] = myvector::VGet(-120, -80, -110);
	////node.Leg2[4] = myvector::VGet(-120, 0, -110);
	////node.Leg2[5] = myvector::VGet(-120, 80, -110);
	//
	////姿勢テイトブライアン角グローバル
	//node.pitch = 0.0;//x軸回転
	//node.roll = 0.0;//y軸回転
	//node.yaw = 0.0;//z軸回転
	//
	////node.center_of_mass = 0;//重心位置int
	////node.kaisou = 295;			//iHX4[1][1][1][1][1][1] = 295	iHX4[2][2][2][2][2][2] = 729	iHX4[1][2][1][2][1][2]=670
	////node.v = 0;//上下ノード(全脚接地)
	//
	////node.COM_type = 0;//重心位置タイプ
	//node.parent = NULL;//親ノードのポインタ
	//node.node_height = 1;//ノード高さ
	////node.debug = 0;//現在運動履歴として使用,前回の脚上下ノード(上下運動をした場合)2桁,前回の動作1桁,前々回の動作1桁
}


void SquareMap(myvector::SVector *mapData, int x, int y, int nMap) 
{
	srand((unsigned int)time(NULL));	//乱数の初期値を変える
	std::mt19937 mt;            // メルセンヌ・ツイスタの32ビット版
	std::random_device rnd;     // 非決定的な乱数生成器
	mt.seed(rnd());
	int n = 6;
	myvector::SVector temp;

	while (n < nMap) 
	{
		bool f = true;
		temp.x = ((int)mt() % (x/2) );
		temp.y = ((int)mt() % (y/2) ) + (y/2-500);
		temp.z = 0;

		for (int i = 0; i < n; ++i) 
		{
			if (myvector::V2Mag2(temp, mapData[i]) < 50) 
			{
				f = false;
				break;
			}
		}
		if (f) 
		{
			mapData[n] = temp;
			++n;
		}
	}
}

//円形にランダム　seed固定
void SquareMap(myvector::SVector *mapData, int x, int y, int nMap, int seed) 
{
	srand(seed);	//乱数の初期値を変える
	int n = 6;
	myvector::SVector temp;

	while (n < nMap) 
	{
		bool f = true;
		temp.x = ((int)rand() % x) - 500;
		temp.y = ((int)rand() % y) - 500;
		temp.z = 0;

		for (int i = 0; i < n; ++i) 
		{
			if (myvector::V2Mag2(temp, mapData[i]) < 50) 
			{
				f = false;
				break;
			}
		}
		if (f) 
		{
			mapData[n] = temp;
			++n;
		}
	}
}

void CircleMap(myvector::SVector *mapData, myvector::SVector center, int Rmin, int Rmax, float amax, float amin,  int nMap, int seed) 
{
	srand(seed);
	float minAngle = amin * (float)Define::MY_PI / 180.0f;
	float maxAngle = amax * (float)Define::MY_PI / 180.0f;
	int n = 6;
	double x,y;
	//p_mapData3D[6] = center;

	while(n <nMap)
	{
		x = rand() % (Rmax * 2) - Rmax;
		y = rand() % (Rmax * 2) - Rmax;
		if(atan2(y,x) < minAngle || atan2(y,x) > maxAngle) continue;//角度で制限
		if(sqrt(x*x + y*y) >= Rmin && sqrt(x*x + y*y) <= Rmax)
		{
			mapData[n].x = x + center.x;				//脚着地可能点
			mapData[n].y = y + center.y;				//脚着地可能点
			mapData[n].z = 0;
			for(int i = 0; i < n; i++)
			{
				//近い脚接地点は使用しない
				if(myvector::V2Mag2(mapData[n],mapData[i])<70)
				{
					break;
				}
				else if(n == (i + 1))
				{
					n++;
					break;
				}
			}
		}
	}
}

void CircleMap(myvector::SVector *mapData, myvector::SVector center, int Rmin, int Rmax, float amax, float amin, int nMap) 
{
	//円形に配置　
	srand((unsigned int)time(NULL));	//乱数の初期値を変える
	//円形にランダムに配置
	std::mt19937 mt;            // メルセンヌ・ツイスタの32ビット版
	std::random_device rnd;     // 非決定的な乱数生成器
	mt.seed( rnd() );
	float minAngle = amin * (float)Define::MY_PI / 180.0f;
	float maxAngle = amax * (float)Define::MY_PI / 180.0f;
	//myvector::VECTOR center = {-1000,0,0};	//円の中心
	int n = 7;
	double x,y;
	mapData[6] = center;

	while(n < nMap)
	{
		x = ((int)mt() - (1 << 31) ) % (Rmax * 2) - Rmax;
		y = ((int)mt() - (1 << 31) ) % (Rmax * 2) - Rmax;
		if(atan2(y,x) < minAngle || atan2(y,x) > maxAngle) continue;//角度で制限
		if(sqrt(x*x + y*y) >= Rmin && sqrt(x*x + y*y) <= Rmax)
		{
			mapData[n].x = x + center.x;				//脚着地可能点
			mapData[n].y = y + center.y;				//脚着地可能点
			mapData[n].z = 0;
			for(int i = 0; i < n; i++)
			{
				//近い脚接地点は使用しない
				if(myvector::V2Mag2(mapData[n],mapData[i])<50)
				{
					//元70
					break;
				}
				else if(n == (i + 1))
				{
					n++;
					break;
				}
			}
		}
	}
}

void GetRandom(int mapDataNum, int min, int max, int* Random)
{
	srand((unsigned int)time(NULL));
	for (int i = 0; i < mapDataNum; i++) 
	{
		Random[i] = min + (int)(rand()*(max - min + 1.0) / (1.0 + RAND_MAX));
		//std::cout << Random[i] << std::endl;
	}
}