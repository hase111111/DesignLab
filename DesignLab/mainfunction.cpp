#include "mainfunction.h"
#include "hexapod.h"
#include "MapDefine.h"
#include <iostream>
#include <random>
#include <string>
#include <sstream>

void getMap(myvector::SVector *p_mapData3D, int* mapData3D_MAX, LNODE* CurrentCondition, const int f) 
{
	*mapData3D_MAX = MapConst::MAPDATA3D_MAX;
	//脚接地候補点の数(変更する場合)最大値はmapData.h参照
	//上式の定数の値を変えた場合は、mapData.hのMAPDATA3D_MAXも同値になるように変更すること。ちょいめんどいけど
	//*mapData3D_MAX = 1000;

	Hexapod phantomX;
	phantomX.setMyPosition(CurrentCondition->global_center_of_mass);//重心位置グローバル
	//phantomX.setTravelingDirection(CurrentCondition->global_center_of_mass);//進行方向(重心位置(0,0,110))
	phantomX.setMyDirection(CurrentCondition->pitch, CurrentCondition->roll, CurrentCondition->yaw);//姿勢(テイトブライアン角)グローバル、初期姿勢が違うときは変更する必要あり//(0,0,0)から変更
	phantomX.setLocalLegPos(CurrentCondition->Leg);
	phantomX.setLocalLeg2Pos(CurrentCondition->Leg2);//付け根から脚先の位置ローカル

	//初期姿勢における足先座標を脚設置可能点とする
	p_mapData3D[0] = phantomX.getGlobalLegPos(0);
	p_mapData3D[1] = phantomX.getGlobalLegPos(1);
	p_mapData3D[2] = phantomX.getGlobalLegPos(2);
	p_mapData3D[3] = phantomX.getGlobalLegPos(3);
	p_mapData3D[4] = phantomX.getGlobalLegPos(4);
	p_mapData3D[5] = phantomX.getGlobalLegPos(5);

	for (int i = 0; i < 6; i++) 
	{
		if (p_mapData3D[i].z >= 25) 
		{
			p_mapData3D[i].x = 10000;
		}
	}
	for (int i = 0; i < 6; i++) { myvector::VectorOutPut(p_mapData3D[i]); }
	for (int i = 0; i < 6; i++) { myvector::VectorOutPut(phantomX.getGlobalLeg2Pos(i)); }

	////////////////////////////////////////////////////////////////////////////
	//個人的な計算用めも	190613時点　phantomX前提
	//脚を伸ばすと脚長は大体190mm	(hexapod.h参照)
	//重心高さは110mm
	//脚高さは80mmだから，遊脚した脚先は地面からは30mmの位置
	//有効な高さ可動範囲は80mm~190mm
	//余裕をもって90~170にすべきか
	//その高さにおける最大到達半径はhexapodクラスのLegROM_r参照
	//最小半径は50固定
	////////////////////////////////////////////////////////////////////////////

	int maxX = MapConst::MAP_X_MIN;
	int maxY = MapConst::MAP_Y_MIN;
	int x_count = 0;
	int y_count = 0;
	int rough_terrain_count = 0;
	int invalid_count = 0;

#ifdef MAKE_SLOPE_MAP
	//勾配ベクトルを参照 z = f(x,y) = x *tan_roll + y * tan_pitch;
	double tan_pitch = sqrt(tan(MapConst::THETA_SLOPE * Define::MY_PI / 180) * tan(MapConst::THETA_SLOPE * Define::MY_PI / 180) / (1 + tan(MapConst::XI_SLOPE * Define::MY_PI / 180) * tan(MapConst::XI_SLOPE * Define::MY_PI / 180)));
	double tan_roll = tan(MapConst::XI_SLOPE * Define::MY_PI / 180) * tan_pitch;
#endif

#ifdef ISOSELES_TRIANGLE
	bool plus = false;
#endif	

	for (int i = 6; i < MapConst::MAPDATA3D_MAX; ++i) 
	{
		maxX += MapConst::FOOT_HOLD_XY_DIST;
		x_count++;
		
		if (maxX >= MapConst::MAP_X_MAX - MapConst::FOOT_HOLD_XY_DIST)
		{
			//yの最小値でxを最小値から最大値まで横に並べていって最大値まで達したら，yの最小値を更新して，ｘをまた最小値から横に並べる．
			maxX = MapConst::MAP_X_MIN;
			maxY += MapConst::FOOT_HOLD_XY_DIST;
			y_count++;

#ifdef ISOSELES_TRIANGLE
			if ((maxY - MapConst::START_ROUGH_TARRAIN_Y) / MapConst::DEPTH_TRI % 2 == 0) { plus = false; }
			else { plus = true; }
#endif
			x_count = 0;
		}

		if (maxY <= MapConst::MAP_Y_MAX - MapConst::FOOT_HOLD_XY_DIST)
		{
			//yがマップの最大値を超すまで脚設置可能点として３次元座標を保存する、この時点ではすべて平面
			p_mapData3D[i].x = maxX;
			p_mapData3D[i].y = maxY;
			p_mapData3D[i].z = 0;//この時点ではすべて水平面

			//↓ここから，不整地の領域に対してz(高さ)を変えて不整地に変更する．
			if (p_mapData3D[i].y > (double)MapConst::START_ROUGH_TARRAIN_Y)
			{
				++rough_terrain_count;
#ifdef MAKE_SLOPE_MAP 
				p_mapData3D[i].z = -maxX * tan_roll - (maxY - MapConst::START_ROUGH_TARRAIN_Y) * tan_pitch;
#endif
#ifdef MAKE_STEP_MAP
				//int型の切り捨てを利用している気がするので,floatやdoubleにキャストするのは危険そう．
				//後で治す
				p_mapData3D[i].z = ((maxY - MapConst::START_ROUGH_TARRAIN_Y) / MapConst::DEPTH_STEP + 1) * MapConst::HEIGHT_STEP;
#endif


#ifdef ISOSELES_TRIANGLE
				if (x_count % (MapConst::WIDE_TRI / MapConst::FOOT_HOLD_XY_DIST) == 0 && plus == true)
				{
					p_mapData3D[i].z = MapConst::WIDE_TRI * tan(MapConst::THETA_TRI*Define::MY_PI / 180);
					plus = false; 
				}
				else if (x_count % (MapConst::WIDE_TRI / MapConst::FOOT_HOLD_XY_DIST) == 0 && plus == false)
				{
					p_mapData3D[i].z = MapConst::WIDE_TRI * tan(MapConst::THETA_TRI*Define::MY_PI / 180);
					p_mapData3D[i].z = 0;
					plus = true;
				}
				else if (x_count % (MapConst::WIDE_TRI / MapConst::FOOT_HOLD_XY_DIST) != 0 && plus == true)
				{
					p_mapData3D[i].z = MapConst::FOOT_HOLD_XY_DIST * (x_count % (MapConst::WIDE_TRI / MapConst::FOOT_HOLD_XY_DIST)) * tan(MapConst::THETA_TRI*Define::MY_PI / 180);
				}
				else if (x_count % (MapConst::WIDE_TRI / MapConst::FOOT_HOLD_XY_DIST) != 0 && plus == false)
				{
					p_mapData3D[i].z = MapConst::WIDE_TRI * tan(MapConst::THETA_TRI*Define::MY_PI / 180) - MapConst::FOOT_HOLD_XY_DIST * (x_count % (MapConst::WIDE_TRI / MapConst::FOOT_HOLD_XY_DIST)) * tan(MapConst::THETA_TRI*Define::MY_PI / 180);
				}
#endif
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
	for (int i = 6; i < MapConst::MAPDATA3D_MAX; ++i) 
	{
		maxX += MapConst::FOOT_HOLD_XY_DIST;
		x_count++;
		if (maxX >= MAP_X_MAX - MapConst::FOOT_HOLD_XY_DIST)
		{
			//yの最小値でxを最小値から最大値まで横に並べていって最大値まで達したら，yの最小値を更新して，ｘをまた最小値から横に並べる．
			maxX = MAP_X_MIN;
			maxY += MapConst::FOOT_HOLD_XY_DIST;
			y_count++;
			x_count = 0;
		}

		if (maxY <= MAP_Y_MAX - MapConst::FOOT_HOLD_XY_DIST)
		{//yがマップの最大値を超すまで脚設置可能点として３次元座標を保存する、この時点ではすべて平面
			if (p_mapData3D[i].y > (double)MapConst::START_ROUGH_TARRAIN_Y)
			{
				if (((maxX - MAP_X_MIN) / MapConst::SQUARE_SIZE) % 2 != 0) 
				{	//縦じま
				//if (((maxY - MAP_Y_MIN) / MapConst::SQUARE_SIZE) % 2 != 0) {	//横じま
				//if (((maxY - MAP_Y_MIN) / MapConst::SQUARE_SIZE + (maxX - MAP_X_MIN) / MapConst::SQUARE_SIZE) % 2 != 0) { //斜めじま
				//if (((maxY - MAP_Y_MIN) / MapConst::SQUARE_SIZE) % 2 != 0&& ((maxX - MAP_X_MIN) / MapConst::SQUARE_SIZE) % 2 != 0) { //網目
				//if (((maxY - MAP_Y_MIN) / MapConst::SQUARE_SIZE) % 2 != 0 || ((maxX - MAP_X_MIN) / MapConst::SQUARE_SIZE) % 2 != 0) {	//格子点（網目の逆）
					p_mapData3D[i].x = MapConst::INVALID_FOOT_HOLD;
					p_mapData3D[i].y = MapConst::INVALID_FOOT_HOLD;
					p_mapData3D[i].z = MapConst::INVALID_FOOT_HOLD;
					++invalid_count;
				}
			}
		}
	}

#endif

	//ランダム足場にする(もともと整地の座標が格納されている前提で使用)
#ifdef HOLE_RANDOM
		int Random1[MapConst::MAPDATA3D_MAX];//乱数を格納する配列1//足場の数を入れる要は*mapData3D_MAXの値
		int Random2[MapConst::MAPDATA3D_MAX];
		int Random3[MapConst::MAPDATA3D_MAX];
		GetRandom(*mapData3D_MAX, 1, 100, Random1);//1~100の乱数を格納//割合%を表す
		GetRandom(*mapData3D_MAX, 0, MapConst::HEIGHT_MAGNIFICATION, Random2);//乱数を格納//とりあえずランダムなずれの倍率を表す
		GetRandom(*mapData3D_MAX, 0, MapConst::MAPDATA3D_MAX - 1, Random3);//乱数を格納//ランダムなインデックス番号//1点ずつ

		for (int i = 6; i < MapConst::MAPDATA3D_MAX; ++i) 
		{
			for (int xx = MapConst::MAP_X_MIN; xx < MapConst::MAP_X_MAX; xx += MapConst::SQUARE_SIZE)
			{
				for (int yy = MapConst::START_ROUGH_TARRAIN_Y; yy < MapConst::MAP_Y_MAX; yy += MapConst::SQUARE_SIZE)
				{
					if (xx <= p_mapData3D[i].x && p_mapData3D[i].x < xx + MapConst::SQUARE_SIZE)
					{
						if (yy <= p_mapData3D[i].y && p_mapData3D[i].y < yy + MapConst::SQUARE_SIZE)
						{
							if (xx < 0) xx += 1050;
							if (yy < 0) yy += 1050;
							if (Random1[Random3[20 * xx / MapConst::SQUARE_SIZE + yy / MapConst::SQUARE_SIZE]] < MapConst::HOLE_RATE) {//足場を無くす割合(%)
								p_mapData3D[i].x = (double)MapConst::INVALID_FOOT_HOLD;//絶対に脚が届かないところに足場を飛ばす
								p_mapData3D[i].y = (double)MapConst::INVALID_FOOT_HOLD;//絶対に脚が届かないところに足場を飛ばす
								p_mapData3D[i].z = (double)MapConst::INVALID_FOOT_HOLD;//絶対に脚が届かないところに足場を飛ばす
							}
#ifdef RANDOM_ADD_Z //高さ方向にも乱数を振りたいときはヘッダーファイルでdefine
							else if (Random1[20 * xx / MapConst::SQUARE_SIZE + yy / MapConst::SQUARE_SIZE] % 2 == 0) {
								p_mapData3D[i].z += Random2[Random3[20 * xx / MapConst::SQUARE_SIZE + yy / MapConst::SQUARE_SIZE]] * RANDOM_ADD_Z;
							}
							else {
								p_mapData3D[i].z -= Random2[Random3[20 * xx / MapConst::SQUARE_SIZE + yy / MapConst::SQUARE_SIZE]] * RANDOM_ADD_Z;
							}
#endif
						}
					}
				}
			}

		}
#endif

	//脚設置可能点のデータをexcelファイルに出力する
	if (f == 0) 
	{
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

}

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

int ReadMapDataFromFile(const std::string& filename, myvector::SVector *map, const char delimiter) 
{
	std::ifstream ifs(filename);//読み込むファイル
	std::string line;//ファイルの行
	int j = 0;

	if (!ifs.is_open()) 
	{
		std::cout << "マップデータファイルが開けません．" << std::endl;
		while (1);
	}

	while (std::getline(ifs, line)) 
	{
		//行ごとに読み込み
		std::istringstream stream(line);//
		std::string field;
		std::vector<std::string> result;

		while (std::getline(stream, field, delimiter)) 
		{
			//行の読み込みが終わるまで
			result.push_back(field);//","ごとにresultにプッシュバック//この時点では文字として入力される
		}

		if (j != 0) {//8行目のマップデータから読み込む　j=0→1行目=変数名 j=1～6→2～7行目＝初期の脚先の座標が脚設置可能点になるから上書きしない
			map[j].x = stod(result.at(1));//stringをdoubleに変換して代入
			map[j].y = stod(result.at(2));
			map[j].z = stod(result.at(3));
		}

		++j;//マップデータの数＝行の数
	}
	return j;//読み込んだマップデータの数を返す．

}

void WriteMapDataToFile(std::ofstream& filename, const myvector::SVector *map, const int* mapData3D_MAX) 
{
	filename << "n" << "," << "x" << "," << "y" << "," << "z" << std::endl;

	for (int i = 0; i < *mapData3D_MAX; ++i) 
	{
		filename << i << "," << map[i].x << "," << map[i].y << "," << map[i].z << "\n";
	}
}

void MapSqrtDivide(myvector::SVector *mapData, int mapDataNum, std::vector< std::vector< std::vector<myvector::SVector> > > &divideMapData, int pointNum[MapConst::LP_DIVIDE_NUM][MapConst::LP_DIVIDE_NUM])
{	
	for (int i = 0; i < MapConst::LP_DIVIDE_NUM; ++i)
	{
		for (int j = 0; j < MapConst::LP_DIVIDE_NUM; ++j)
		{
			pointNum[i][j] = 0;	//ゼロクリア
		}
	}

	//脚接地可能点の存在している範囲 //引数にすべきか考え中
	double xMax = (double)MapConst::MAP_X_MAX;
	double xMin = (double)MapConst::MAP_X_MIN;
	double yMax = (double)MapConst::MAP_Y_MAX;
	double yMin = (double)MapConst::MAP_Y_MIN;

	double lengthX = (xMax - xMin) / (double)MapConst::LP_DIVIDE_NUM;	//1ブロックの長さ いまは２０
	double lengthY = (yMax - yMin) / (double)MapConst::LP_DIVIDE_NUM;//いまは100

	int x, y, t;
	for (int i = 0; i < mapDataNum; ++i) 
	{
		if (mapData[i].x == MapConst::INVALID_FOOT_HOLD) continue;
		x = (int)((mapData[i].x - xMin) / lengthX);		//x方向のブロック番号
		y = (int)((mapData[i].y - yMin) / lengthY);		//y方向のブロック番号
		
		if (x >= MapConst::LP_DIVIDE_NUM) { x = MapConst::LP_DIVIDE_NUM - 1; }	//想定外の範囲の場合は端っこのブロックに収めるようにする
		if (x < 0) { x = 0; }
		if (y >= MapConst::LP_DIVIDE_NUM) { y = MapConst::LP_DIVIDE_NUM - 1; }
		if (y < 0) { y = 0; }

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
	double xMax = (double)MapConst::MAP_X_MAX;
	double xMin = (double)MapConst::MAP_X_MIN;
	double yMax = (double)MapConst::MAP_Y_MAX;
	double yMin = (double)MapConst::MAP_Y_MIN;

	double lengthX = (xMax - xMin) / (double)MapConst::LP_DIVIDE_NUM;	//1ブロックの長さ
	double lengthY = (yMax - yMin) / (double)MapConst::LP_DIVIDE_NUM;

	//ブロック番号計算
	x1 = (int)((p1.x - xMin) / lengthX);
	x2 = (int)((p2.x - xMin) / lengthX);
	y1 = (int)((p1.y - yMin) / lengthY);
	y2 = (int)((p2.y - yMin) / lengthY);
	
	if (x1 >= MapConst::LP_DIVIDE_NUM) { x1 = MapConst::LP_DIVIDE_NUM - 1; }//想定外の範囲の場合は端っこのブロックに収めるようにする
	if (x1 < 0) { x1 = 0; }
	if (y1 >= MapConst::LP_DIVIDE_NUM) { y1 = MapConst::LP_DIVIDE_NUM - 1; }
	if (y1 < 0) { y1 = 0; }
	if (x2 >= MapConst::LP_DIVIDE_NUM) { x2 = MapConst::LP_DIVIDE_NUM - 1; }	//想定外の範囲の場合は端っこのブロックに収めるようにする
	if (x2 < 0) { x2 = 0; }
	if (y2 >= MapConst::LP_DIVIDE_NUM) { y2 = MapConst::LP_DIVIDE_NUM - 1; }
	if (y2 < 0) { y2 = 0; }
}

void SetConditionForStripe(LNODE &node,const int f) 
{
	double COM_Z = 130;
	int random_r[MapConst::START_RANDOM_R];
	int random_theta[360];
	int random_l[1000];
	GetRandom(MapConst::START_RANDOM_R, 0, MapConst::START_RANDOM_R, random_r);
	GetRandom(360, 0, 360, random_theta);
	GetRandom(1000, 0, 1000, random_l);

	for (int i = 0; i < 1000; ++i) 
	{
		if (random_l[i] > 500) { random_l[i] -= 1000; }
	}

	//重心位置グローバル
	node.global_center_of_mass = myvector::VGet(random_l[1] + random_r[1] * cos(random_theta[1] * Define::MY_PI / 180), random_r[1] * sin(random_theta[1] * Define::MY_PI / 180), COM_Z);

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
	node.pitch = 0.0;		//x軸回転
	node.roll = 0.0;		//y軸回転
	node.yaw = 0.0;			//z軸回転
	node.center_of_mass = 0;//重心位置int
	node.leg_state = 0b00000000110011001100110011001100;
	node.parent = NULL;		//親ノードのポインタ
	node.node_height = 1;	//ノード高さ
	node.debug = 24;		//現在運動履歴として使用,前回の脚上下ノード(上下運動をした場合)2桁,前回の動作1桁,前々回の動作1桁
	node.delta_comz = 0;
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
