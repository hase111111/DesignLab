#include "simulation_map_creator.h"

#include <algorithm>
#include <random>

#include "cassert_define.h"
#include "designlab_math_util.h"
#include "map_const.h"


namespace dlm = designlab::math_util;


SimulationMapCreator::SimulationMapCreator(MapCreateMode mode, unsigned int option) : 
	mode_(mode),
	option_(option)
{
}

MapState SimulationMapCreator::InitMap()
{
	std::vector<designlab::Vector3> map_data;

	switch (mode_)
	{
	case MapCreateMode::kFlat:
	{
		CreateFlatMap(&map_data);
		break;
	}
	case MapCreateMode::kVerticalStripe:
	{
		CreateVerticalStripeMap(&map_data);
		break;
	}
	case MapCreateMode::kHorizontalStripe:
	{
		CreateHorizontalStripeMap(&map_data);
		break;
	}
	case MapCreateMode::kDiagonalStripe:
	{
		CreateDiagonalStripeMap(&map_data);
		break;
	}
	case MapCreateMode::kMesh:
	{
		CreateMeshMap(&map_data);
		break;
	}
	case MapCreateMode::kLatticePoint:
	{
		CreateLatticePointMap(&map_data);
		break;
	}
	default:
	{
		//異常な値が入力されたら，平面のマップを生成する．
		CreateFlatMap(&map_data);
		break;
	}

	}

	//オプション指定に基づき，Z座標を変更する

	if (option_ & static_cast<unsigned int>(MapCreateOption::kPerforated))
	{
		//穴あき地形にする．
		ChangeMapToPerforated(&map_data);
	}

	if (option_ & static_cast<unsigned int>(MapCreateOption::kStep))
	{
		//階段状にする．
		ChangeMapToStep(&map_data);
	}

	if (option_ & static_cast<unsigned int>(MapCreateOption::kSlope))
	{
		//坂道にする．
		ChangeMapToSlope(&map_data);
	}

	if (option_ & static_cast<unsigned int>(MapCreateOption::kTilt))
	{
		//坂道にする．
		ChangeMapToTilt(&map_data);
	}

	if (option_ & static_cast<unsigned int>(MapCreateOption::kRough))
	{
		//デコボコにする．
		ChangeMapToRough(&map_data);
	}

	MapState res;

	res.SetMapPointVec(map_data);

	return res;
}

void SimulationMapCreator::UpdateMap([[maybe_unused]]MapState* current_map)
{
	//マップを更新する必要がないので，何もしない．
}


void SimulationMapCreator::CreateFlatMap(std::vector<designlab::Vector3>* map) const
{
	assert(map != nullptr);		//mapがnullptrでないことを確認する．
	assert(map->empty());		//mapが空であることを確認する．

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

void SimulationMapCreator::CreateVerticalStripeMap(std::vector<designlab::Vector3>* map) const
{
	assert(map != nullptr);		//mapがnullptrでないことを確認する．
	assert(map->empty());		//mapが空であることを確認する．

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

void SimulationMapCreator::CreateHorizontalStripeMap(std::vector<designlab::Vector3>* map) const
{
	assert(map != nullptr);		//mapがnullptrでないことを確認する．
	assert(map->empty());		//mapが空であることを確認する．

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

void SimulationMapCreator::CreateDiagonalStripeMap(std::vector<designlab::Vector3>* map) const
{
	assert(map != nullptr);		//mapがnullptrでないことを確認する．
	assert(map->empty());		//mapが空であることを確認する．

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

void SimulationMapCreator::CreateMeshMap(std::vector<designlab::Vector3>* map) const
{
	assert(map != nullptr);		//mapがnullptrでないことを確認する．
	assert(map->empty());		//mapが空であることを確認する．

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

void SimulationMapCreator::CreateLatticePointMap(std::vector<designlab::Vector3>* map) const
{
	assert(map != nullptr);		//mapがnullptrでないことを確認する．
	assert(map->empty());		//mapが空であることを確認する．

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

void SimulationMapCreator::ChangeMapToPerforated(std::vector<designlab::Vector3>* map) const
{
	assert(map != nullptr);		//mapがnullptrでないことを確認する．

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

void SimulationMapCreator::ChangeMapToStep(std::vector<designlab::Vector3>* map) const
{
	assert(map != nullptr);		//mapがnullptrでないことを確認する．

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

void SimulationMapCreator::ChangeMapToSlope(std::vector<designlab::Vector3>* map) const
{
	assert(map != nullptr);		//mapがnullptrでないことを確認する．

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

void SimulationMapCreator::ChangeMapToTilt(std::vector<designlab::Vector3>* map) const
{
	assert(map != nullptr);		//mapがnullptrでないことを確認する．

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

void SimulationMapCreator::ChangeMapToRough(std::vector<designlab::Vector3>* map) const
{
	assert(map != nullptr);		//mapがnullptrでないことを確認する．

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
