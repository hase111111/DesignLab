#include "map_creator.h"

#include <iostream>
#include <random>
#include <algorithm>
#include <string>
#include <fstream>
#include <sstream>

#include "map_const.h"
#include "designlab_math_util.h"


namespace dlm = designlab::math_util;


MapState MapCreator::Create(const MapCreateMode mode, const int option)
{
	std::vector<designlab::Vector3> map_data;

	switch (mode)
	{
	case MapCreateMode::kFlat:
	{
		createFlatMap(&map_data);
		break;
	}
	case MapCreateMode::VERTICAL_STRIPE:
	{
		createVerticalStripeMap(&map_data);
		break;
	}
	case MapCreateMode::HORIZONTAL_STRIPE:
	{
		createHorizontalStripeMap(&map_data);
		break;
	}
	case MapCreateMode::DIAGONAL_STRIPE:
	{
		createDiagonalStripeMap(&map_data);
		break;
	}
	case MapCreateMode::MESH:
	{
		createMeshMap(&map_data);
		break;
	}
	case MapCreateMode::LATTICE_POINT:
	{
		createLatticePointMap(&map_data);
		break;
	}
	case MapCreateMode::READ_FROM_FILE:

		//ファイルからマップを生成する．
		if (!createMapFromFile(&map_data))
		{
			//読み込みに失敗したら，平面のマップを生成する．
			createFlatMap(&map_data);
		}
		break;

	default:

		//異常な値が入力されたら，平面のマップを生成する．
		createFlatMap(&map_data);
		break;
	}

	//オプション指定に基づき，Z座標を変更する
	if (option & OPTION_PERFORATED && mode != MapCreateMode::READ_FROM_FILE)
	{
		//穴あき地形にする．
		changeMapPerforated(&map_data);
	}

	if (option & OPTION_STEP && mode != MapCreateMode::READ_FROM_FILE)
	{
		//階段状にする．
		changeMapStep(&map_data);
	}

	if (option & OPTION_SLOPE && mode != MapCreateMode::READ_FROM_FILE)
	{
		//坂道にする．
		changeMapSlope(&map_data);
	}

	if (option & OPTION_TILT && mode != MapCreateMode::READ_FROM_FILE)
	{
		//坂道にする．
		changeMapTilt(&map_data);
	}

	if (option & OPTION_ROUGH && mode != MapCreateMode::READ_FROM_FILE)
	{
		//デコボコにする．
		changeMapRough(&map_data);
	}

	MapState res;

	res.SetMapPoint(map_data);

	return res;
}

void MapCreator::PrintAllMapCreateMode()
{
	std::cout << "MapCreator.h : MapCreator::PrintAllMapCreateMode() " << std::endl;
	std::cout << "FLAT : " << static_cast<int>(MapCreateMode::kFlat) << std::endl;
	std::cout << "VERTICAL_STRIPE : " << static_cast<int>(MapCreateMode::VERTICAL_STRIPE) << std::endl;
	std::cout << "HORIZONTAL_STRIPE : " << static_cast<int>(MapCreateMode::HORIZONTAL_STRIPE) << std::endl;
	std::cout << "DIAGONAL_STRIPE : " << static_cast<int>(MapCreateMode::DIAGONAL_STRIPE) << std::endl;
	std::cout << "MESH : " << static_cast<int>(MapCreateMode::MESH) << std::endl;
	std::cout << "LATTICE_POINT : " << static_cast<int>(MapCreateMode::LATTICE_POINT) << std::endl;
	std::cout << "READ_FROM_FILE : " << static_cast<int>(MapCreateMode::READ_FROM_FILE) << std::endl;
}

void MapCreator::printAllMapCreateOption()
{
	std::cout << "MapCreator.h : MapCreator::printAllMapCreateOption() " << std::endl;
	std::cout << "OPTION_NONE : " << MapCreator::OPTION_NONE << std::endl;
	std::cout << "OPTION_PERFORATED : " << MapCreator::OPTION_PERFORATED << std::endl;
	std::cout << "OPTION_STEP : " << MapCreator::OPTION_STEP << std::endl;
	std::cout << "OPTION_SLOPE : " << MapCreator::OPTION_SLOPE << std::endl;
	std::cout << "OPTION_TILT : " << MapCreator::OPTION_TILT << std::endl;
	std::cout << "OPTION_ROUGH : " << MapCreator::OPTION_ROUGH << std::endl;
	std::cout << "複数指定したい場合は，指定したいオプションを足し合わせてください．" << std::endl;
}


void MapCreator::createFlatMap(std::vector<designlab::Vector3>* map)
{
	(*map).clear();	//マップを初期化する.

	// マップの xとyの存在範囲全体に脚設置可能点を敷き詰める．
	for (int x = 0; x < (MapConst::MAP_MAX_FORWARD - MapConst::MAP_MIN_FORWARD) / MapConst::FOOT_HOLD_XY_DIST; x++)
	{
		for (int y = 0; y < (MapConst::MAP_MAX_HORIZONTAL - MapConst::MAP_MIN_HORIZONTAL) / MapConst::FOOT_HOLD_XY_DIST; y++)
		{
			const float x_pos = MapConst::MAP_MIN_FORWARD + (float)x * MapConst::FOOT_HOLD_XY_DIST;	//ロボットの正面方向．
			const float y_pos = MapConst::MAP_MIN_HORIZONTAL + (float)y * MapConst::FOOT_HOLD_XY_DIST;	//ロボットの側面方向．

			(*map).push_back(designlab::Vector3(x_pos, y_pos, MapConst::MAX_Z_BASE));	//脚設置可能点を追加する．
		}
	}
}

void MapCreator::createVerticalStripeMap(std::vector<designlab::Vector3>* map)
{
	(*map).clear();	//マップを初期化する.

	// マップの xとyの存在範囲全体に脚設置可能点を敷き詰める．
	for (int x = 0; x < (MapConst::MAP_MAX_FORWARD - MapConst::MAP_MIN_FORWARD) / MapConst::FOOT_HOLD_XY_DIST; x++)
	{
		for (int y = 0; y < (MapConst::MAP_MAX_HORIZONTAL - MapConst::MAP_MIN_HORIZONTAL) / MapConst::FOOT_HOLD_XY_DIST; y++)
		{
			//縦じまをつくるために，一定間隔ごとに追加する．最初の待機場所の座標ならば無条件に追加する
			if (y % (MapConst::STRIPE_INTERVAL * 2) < MapConst::STRIPE_INTERVAL || x < (MapConst::MAP_START_ROUGH - MapConst::MAP_MIN_FORWARD) / MapConst::FOOT_HOLD_XY_DIST)
			{
				const float x_pos = MapConst::MAP_MIN_FORWARD + (float)x * MapConst::FOOT_HOLD_XY_DIST;	//ロボットの正面方向．
				const float y_pos = MapConst::MAP_MIN_HORIZONTAL + (float)y * MapConst::FOOT_HOLD_XY_DIST;	//ロボットの側面方向．

				(*map).push_back(designlab::Vector3(x_pos, y_pos, MapConst::MAX_Z_BASE));	//脚設置可能点を追加する．
			}
		}
	}
}

void MapCreator::createHorizontalStripeMap(std::vector<designlab::Vector3>* map)
{
	(*map).clear();	//マップを初期化する.

	// マップの xとyの存在範囲全体に脚設置可能点を敷き詰める．
	for (int x = 0; x < (MapConst::MAP_MAX_FORWARD - MapConst::MAP_MIN_FORWARD) / MapConst::FOOT_HOLD_XY_DIST; x++)
	{
		for (int y = 0; y < (MapConst::MAP_MAX_HORIZONTAL - MapConst::MAP_MIN_HORIZONTAL) / MapConst::FOOT_HOLD_XY_DIST; y++)
		{
			//縦じまをつくるために，一定間隔ごとに追加する．最初の待機場所の座標ならば無条件に追加する
			if (x % (MapConst::STRIPE_INTERVAL * 2) < MapConst::STRIPE_INTERVAL || x < (MapConst::MAP_START_ROUGH - MapConst::MAP_MIN_FORWARD) / MapConst::FOOT_HOLD_XY_DIST)
			{
				const float x_pos = MapConst::MAP_MIN_FORWARD + (float)x * MapConst::FOOT_HOLD_XY_DIST;	//ロボットの正面方向．
				const float y_pos = MapConst::MAP_MIN_HORIZONTAL + (float)y * MapConst::FOOT_HOLD_XY_DIST;	//ロボットの側面方向．

				(*map).push_back(designlab::Vector3(x_pos, y_pos, MapConst::MAX_Z_BASE));	//脚設置可能点を追加する．
			}
		}
	}
}

void MapCreator::createDiagonalStripeMap(std::vector<designlab::Vector3>* map)
{
	(*map).clear();	//マップを初期化する.

	// マップの xとyの存在範囲全体に脚設置可能点を敷き詰める．
	for (int x = 0; x < (MapConst::MAP_MAX_FORWARD - MapConst::MAP_MIN_FORWARD) / MapConst::FOOT_HOLD_XY_DIST; x++)
	{
		for (int y = 0; y < (MapConst::MAP_MAX_HORIZONTAL - MapConst::MAP_MIN_HORIZONTAL) / MapConst::FOOT_HOLD_XY_DIST; y++)
		{
			//斜めじまをつくるために，一定間隔ごとに追加する．最初の待機場所の座標ならば無条件に追加する
			const bool _do_create_map = (y % (MapConst::STRIPE_INTERVAL * 2) < MapConst::STRIPE_INTERVAL) ^ (x % (MapConst::STRIPE_INTERVAL * 2) < MapConst::STRIPE_INTERVAL);

			if (_do_create_map || x < (MapConst::MAP_START_ROUGH - MapConst::MAP_MIN_FORWARD) / MapConst::FOOT_HOLD_XY_DIST)
			{
				const float x_pos = MapConst::MAP_MIN_FORWARD + (float)x * MapConst::FOOT_HOLD_XY_DIST;	//ロボットの正面方向．
				const float y_pos = MapConst::MAP_MIN_HORIZONTAL + (float)y * MapConst::FOOT_HOLD_XY_DIST;	//ロボットの側面方向．

				(*map).push_back(designlab::Vector3(x_pos, y_pos, MapConst::MAX_Z_BASE));	//脚設置可能点を追加する．
			}
		}
	}
}

void MapCreator::createMeshMap(std::vector<designlab::Vector3>* map)
{
	(*map).clear();	//マップを初期化する.

	// マップの xとyの存在範囲全体に脚設置可能点を敷き詰める．
	for (int x = 0; x < (MapConst::MAP_MAX_FORWARD - MapConst::MAP_MIN_FORWARD) / MapConst::FOOT_HOLD_XY_DIST; x++)
	{
		for (int y = 0; y < (MapConst::MAP_MAX_HORIZONTAL - MapConst::MAP_MIN_HORIZONTAL) / MapConst::FOOT_HOLD_XY_DIST; y++)
		{
			//網目模様をつくるために，一定間隔ごとに追加する．最初の待機場所の座標ならば無条件に追加する
			bool do_create_map;

			if ((x % (MapConst::STRIPE_INTERVAL * 2) < MapConst::STRIPE_INTERVAL))
			{
				if ((y % (MapConst::STRIPE_INTERVAL * 2) < MapConst::STRIPE_INTERVAL))
				{
					do_create_map = true;
				}
				else
				{
					do_create_map = false;
				}
			}
			else
			{
				do_create_map = true;
			}

			if (do_create_map || x < (MapConst::MAP_START_ROUGH - MapConst::MAP_MIN_FORWARD) / MapConst::FOOT_HOLD_XY_DIST)
			{
				const float x_pos = MapConst::MAP_MIN_FORWARD + (float)x * MapConst::FOOT_HOLD_XY_DIST;	//ロボットの正面方向．
				const float y_pos = MapConst::MAP_MIN_HORIZONTAL + (float)y * MapConst::FOOT_HOLD_XY_DIST;	//ロボットの側面方向．

				(*map).push_back(designlab::Vector3(x_pos, y_pos, MapConst::MAX_Z_BASE));	//脚設置可能点を追加する．
			}
		}
	}
}

void MapCreator::createLatticePointMap(std::vector<designlab::Vector3>* map)
{
	(*map).clear();	//マップを初期化する.

	// マップの xとyの存在範囲全体に脚設置可能点を敷き詰める．
	for (int x = 0; x < (MapConst::MAP_MAX_FORWARD - MapConst::MAP_MIN_FORWARD) / MapConst::FOOT_HOLD_XY_DIST; x++)
	{
		for (int y = 0; y < (MapConst::MAP_MAX_HORIZONTAL - MapConst::MAP_MIN_HORIZONTAL) / MapConst::FOOT_HOLD_XY_DIST; y++)
		{
			//網目模様をつくるために，一定間隔ごとに追加する．最初の待機場所の座標ならば無条件に追加する
			bool do_create_map;

			if ((x % (MapConst::STRIPE_INTERVAL * 2) < MapConst::STRIPE_INTERVAL))
			{
				if ((y % (MapConst::STRIPE_INTERVAL * 2) >= MapConst::STRIPE_INTERVAL))
				{
					do_create_map = false;
				}
				else
				{
					do_create_map = true;
				}
			}
			else
			{
				do_create_map = false;
			}

			if (do_create_map || x < (MapConst::MAP_START_ROUGH - MapConst::MAP_MIN_FORWARD) / MapConst::FOOT_HOLD_XY_DIST)
			{
				const float x_pos = MapConst::MAP_MIN_FORWARD + (float)x * MapConst::FOOT_HOLD_XY_DIST;	//ロボットの正面方向．
				const float y_pos = MapConst::MAP_MIN_HORIZONTAL + (float)y * MapConst::FOOT_HOLD_XY_DIST;	//ロボットの側面方向．

				(*map).push_back(designlab::Vector3(x_pos, y_pos, MapConst::MAX_Z_BASE));	//脚設置可能点を追加する．
			}
		}
	}
}

bool MapCreator::createMapFromFile(std::vector<designlab::Vector3>* map)
{
	//マップデータを初期化する
	(*map).clear();

	std::ifstream ifs(MapConst::INPUT_FILE_NAME);	//読み込むファイル
	std::string line;								//ファイルの行

	//ファイルを開くことに失敗したら
	if (!ifs.is_open())
	{
		std::cout << "マップデータファイルが開けません．" << std::endl;
		return false;
	}

	// 1行目には数値がないため，読み込まない
	bool is_first_line = true;

	// 1行ごとに読み込む
	while (std::getline(ifs, line))
	{
		std::istringstream stream(line);
		std::string field;
		std::vector<std::string> result;

		while (std::getline(stream, field, ','))
		{
			result.push_back(field);	//","ごとにresultにプッシュバック．この時点では文字として入力される
		}

		if (!is_first_line)
		{
			//stringをfloatに変換して代入
			designlab::Vector3 _pos(stof(result.at(1)), stof(result.at(2)), stof(result.at(3)));
			(*map).push_back(_pos);
		}
		else { is_first_line = false; }

	}

	ifs.close();

	return true;
}

void MapCreator::changeMapPerforated(std::vector<designlab::Vector3>* map)
{
	//厳密にホール率に合わせるために，まずはマップをSTRIPE_INTERVALにあわせて区切って，全部で何マスあるか調べる．
	const int kCellNumX = (MapConst::MAP_MAX_FORWARD - MapConst::MAP_START_ROUGH) / MapConst::FOOT_HOLD_XY_DIST / MapConst::STRIPE_INTERVAL;
	const int kCellNumY = (MapConst::MAP_MAX_HORIZONTAL - MapConst::MAP_MIN_HORIZONTAL) / MapConst::FOOT_HOLD_XY_DIST / MapConst::STRIPE_INTERVAL;
	const int kCellNum = kCellNumX * kCellNumY;

	//マスの数だけ要素を持つvectorを用意する．値は全てfalseで初期化する．
	std::vector<bool> do_perforated(kCellNum, false);

	//ホール率に合わせて，値をtrueに変更する
	const int kHoleNum = kCellNum * MapConst::HOLE_RATE / 100;

	for (int i = 0; i < kHoleNum; i++)
	{
		do_perforated.at(i) = true;
	}

	//ランダムなホールにするために要素の順番をシャッフルする．
	std::shuffle(std::begin(do_perforated), std::end(do_perforated), std::default_random_engine());

	//マップに穴をあける
	for (auto itr = (*map).begin(); itr != (*map).end();)
	{
		//待機場所の外に対してのみ作業をする
		if ((*itr).x < MapConst::MAP_START_ROUGH)
		{
			itr++;
			continue;
		}

		//マスで区切るとどこに位置するかを調べる．
		const int kCellPosX = (int)((*itr).x - MapConst::MAP_START_ROUGH) / MapConst::FOOT_HOLD_XY_DIST / MapConst::STRIPE_INTERVAL;
		const int kCellPosY = (int)((*itr).y - MapConst::MAP_MIN_HORIZONTAL) / MapConst::FOOT_HOLD_XY_DIST / MapConst::STRIPE_INTERVAL;
		const int kCellPos = kCellPosX * kCellNumY + kCellPosY;

		// cell_posの値がおかしくないかチェックする
		if (0 <= kCellPos && kCellPos < do_perforated.size())
		{
			//穴あけをする場所ならば
			if (do_perforated[kCellPos])
			{
				//脚設置可能点を消してイテレータを更新する
				itr = (*map).erase(itr);
			}
			else
			{
				//消さないならば次へ移動する
				itr++;
			}
		}
		else
		{
			//消さずに次へ移動する
			itr++;
		}
	}
}

void MapCreator::changeMapStep(std::vector<designlab::Vector3>* map)
{
	for (auto& i : *map)
	{
		//待機場所の外に対してのみ作業をする
		if (i.x > MapConst::MAP_START_ROUGH)
		{
			//階段の何段目かを計算する．待機場所のすぐ上が1段目なので1を足している
			const int _step_count = 1 + (int)((i.x - MapConst::MAP_START_ROUGH) / MapConst::STEP_LENGTH);

			//階段状にZ座標を変更する
			i.z += (float)MapConst::STEP_HEIGHT * _step_count;
		}
	}
}

void MapCreator::changeMapSlope(std::vector<designlab::Vector3>* map)
{
	for (auto& i : *map)
	{
		//待機場所の外に対してのみ作業をする
		if (i.x > MapConst::MAP_START_ROUGH)
		{
			//階段状にZ座標を変更する
			i.z += (i.x - MapConst::MAP_START_ROUGH) * tan(MapConst::SLOPE_ANGLE / 180 * dlm::kFloatPi);
		}
	}
}

void MapCreator::changeMapTilt(std::vector<designlab::Vector3>* map)
{
	for (auto& i : *map)
	{
		//待機場所の外に対してのみ作業をする
		if (i.x > MapConst::MAP_START_ROUGH)
		{
			//階段状にZ座標を変更する
			i.z += i.y * tan(MapConst::TILT_ANGLE / 180 * dlm::kFloatPi);
		}
	}
}

void MapCreator::changeMapRough(std::vector<designlab::Vector3>* map)
{
	//まずはマップをSTRIPE_INTERVALにあわせて区切って，全部で何マスあるか調べる．
	const int kCellNumX = (MapConst::MAP_MAX_FORWARD - MapConst::MAP_START_ROUGH) / MapConst::FOOT_HOLD_XY_DIST / MapConst::STRIPE_INTERVAL;
	const int kCellNumY = (MapConst::MAP_MAX_HORIZONTAL - MapConst::MAP_MIN_HORIZONTAL) / MapConst::FOOT_HOLD_XY_DIST / MapConst::STRIPE_INTERVAL;
	const int kCellNum = kCellNumX * kCellNumY;

	//マスの数だけ要素を持つvectorを用意する．
	std::vector<float> change_z_lenght;

	for (int i = 0; i < kCellNum; i++)
	{
		//ランダムなZ座標を入れる．
		change_z_lenght.push_back(dlm::GenerateRandomNumber(MapConst::ROUGH_MIN_HEIGHT, MapConst::ROUGH_MAX_HEIGHT));
	}

	for (auto& i : *map)
	{
		//マスで区切るとどこに位置するかを調べる．
		const int kCellPosX = (int)(i.x - MapConst::MAP_START_ROUGH) / MapConst::FOOT_HOLD_XY_DIST / MapConst::STRIPE_INTERVAL;
		const int kCellPosY = (int)(i.y - MapConst::MAP_MIN_HORIZONTAL) / MapConst::FOOT_HOLD_XY_DIST / MapConst::STRIPE_INTERVAL;
		const int kCellPos = kCellPosX * kCellNumY + kCellPosY;

		// cell_posの値がおかしくないかチェックする
		if (0 <= kCellPos && kCellPos < change_z_lenght.size())
		{
			i.z += change_z_lenght.at(kCellPos);
		}
	}
}
