#include "simulation_map_creator.h"

#include <algorithm>
#include <random>

#include "cassert_define.h"
#include "designlab_math_util.h"


namespace dl = ::designlab;
namespace dlm = ::designlab::math_util;


SimulationMapCreator::SimulationMapCreator(const MapCreateModeMessenger& messanger) :
	messanger_(messanger)
{
	assert(messanger_.map_min_x < messanger_.map_max_x);		//map_min_xがmap_max_xより小さいことを確認する．
	assert(messanger_.map_min_y < messanger_.map_max_y);		//map_min_yがmap_max_yより小さいことを確認する．
}

MapState SimulationMapCreator::InitMap()
{
	using EnumMode = MapCreateModeMessenger::MapCreateMode;
	using EnumOption = MapCreateModeMessenger::MapCreateOption;

	std::vector<dl::Vector3> map_data;

	switch (messanger_.mode)
	{
	case EnumMode::kFlat:
	{
		CreateFlatMap(&map_data);
		break;
	}
	case EnumMode::kVerticalStripe:
	{
		CreateVerticalStripeMap(&map_data);
		break;
	}
	case EnumMode::kHorizontalStripe:
	{
		CreateHorizontalStripeMap(&map_data);
		break;
	}
	case EnumMode::kDiagonalStripe:
	{
		CreateDiagonalStripeMap(&map_data);
		break;
	}
	case EnumMode::kMesh:
	{
		CreateMeshMap(&map_data);
		break;
	}
	case EnumMode::kLatticePoint:
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

	if (messanger_.option & static_cast<unsigned int>(EnumOption::kPerforated))
	{
		//穴あき地形にする．
		ChangeMapToPerforated(&map_data);
	}

	if (messanger_.option & static_cast<unsigned int>(EnumOption::kStep))
	{
		//階段状にする．
		ChangeMapToStep(&map_data);
	}

	if (messanger_.option & static_cast<unsigned int>(EnumOption::kSlope))
	{
		//坂道にする．
		ChangeMapToSlope(&map_data);
	}

	if (messanger_.option & static_cast<unsigned int>(EnumOption::kTilt))
	{
		//坂道にする．
		ChangeMapToTilt(&map_data);
	}

	if (messanger_.option & static_cast<unsigned int>(EnumOption::kRough))
	{
		//デコボコにする．
		ChangeMapToRough(&map_data);
	}

	return MapState(map_data);
}

void SimulationMapCreator::UpdateMap([[maybe_unused]]MapState* current_map)
{
	//マップを更新する必要がないので，何もしない．
}


void SimulationMapCreator::CreateFlatMap(std::vector<dl::Vector3>* map) const
{
	assert(map != nullptr);		//mapがnullptrでないことを確認する．
	assert(map->empty());		//mapが空であることを確認する．
	
	// マップの xとyの存在範囲全体に脚設置可能点を敷き詰める．
	for (int x = 0; x < (messanger_.map_max_x - messanger_.map_min_x) / MapState::kMapPointDistance; x++)
	{
		for (int y = 0; y < (messanger_.map_max_y - messanger_.map_min_y) / MapState::kMapPointDistance; y++)
		{
			const float x_pos = messanger_.map_min_x + x * MapState::kMapPointDistance;	//ロボットの正面方向．
			const float y_pos = messanger_.map_min_y + y * MapState::kMapPointDistance;	//ロボットの側面方向．

			map->push_back({ x_pos, y_pos, messanger_.base_z });	//脚設置可能点を追加する．
		}
	}
}

void SimulationMapCreator::CreateVerticalStripeMap(std::vector<dl::Vector3>* map) const
{
	assert(map != nullptr);		//mapがnullptrでないことを確認する．
	assert(map->empty());		//mapが空であることを確認する．

	// マップの xとyの存在範囲全体に脚設置可能点を敷き詰める．
	for (int x = 0; x < (messanger_.map_max_x - messanger_.map_min_x) / MapState::kMapPointDistance; x++)
	{
		for (int y = 0; y < (messanger_.map_max_y - messanger_.map_min_y) / MapState::kMapPointDistance; y++)
		{
			//縦じまをつくるために，一定間隔ごとに追加する．最初の待機場所の座標ならば無条件に追加する
			if (y % (messanger_.stripe_interval * 2) < messanger_.stripe_interval || 
				x < (messanger_.map_start_rough_x - messanger_.map_min_x) / MapState::kMapPointDistance)
			{
				const float x_pos = messanger_.map_min_x + x * MapState::kMapPointDistance;	//ロボットの正面方向．
				const float y_pos = messanger_.map_min_y + y * MapState::kMapPointDistance;	//ロボットの側面方向．

				map->push_back({ x_pos, y_pos, messanger_.base_z });	//脚設置可能点を追加する．
			}
		}
	}
}

void SimulationMapCreator::CreateHorizontalStripeMap(std::vector<dl::Vector3>* map) const
{
	assert(map != nullptr);		//mapがnullptrでないことを確認する．
	assert(map->empty());		//mapが空であることを確認する．

	// マップの xとyの存在範囲全体に脚設置可能点を敷き詰める．
	for (int x = 0; x < (messanger_.map_max_x - messanger_.map_min_x) / MapState::kMapPointDistance; x++)
	{
		for (int y = 0; y < (messanger_.map_max_y - messanger_.map_min_y) / MapState::kMapPointDistance; y++)
		{
			//縦じまをつくるために，一定間隔ごとに追加する．最初の待機場所の座標ならば無条件に追加する
			if (x % (messanger_.stripe_interval * 2) < messanger_.stripe_interval || 
				x < (messanger_.map_start_rough_x - messanger_.map_min_x) / MapState::kMapPointDistance)
			{
				const float x_pos = messanger_.map_min_x + x * MapState::kMapPointDistance;	//ロボットの正面方向．
				const float y_pos = messanger_.map_min_y + y * MapState::kMapPointDistance;	//ロボットの側面方向．

				map->push_back({ x_pos, y_pos, messanger_.base_z });	//脚設置可能点を追加する．
			}
		}
	}
}

void SimulationMapCreator::CreateDiagonalStripeMap(std::vector<dl::Vector3>* map) const
{
	assert(map != nullptr);		//mapがnullptrでないことを確認する．
	assert(map->empty());		//mapが空であることを確認する．

	// マップの xとyの存在範囲全体に脚設置可能点を敷き詰める．
	for (int x = 0; x < (messanger_.map_max_x - messanger_.map_min_x) / MapState::kMapPointDistance; x++)
	{
		for (int y = 0; y < (messanger_.map_max_y - messanger_.map_min_y) / MapState::kMapPointDistance; y++)
		{
			//斜めじまをつくるために，一定間隔ごとに追加する．最初の待機場所の座標ならば無条件に追加する
			const bool x_in_stripe = x % (messanger_.stripe_interval * 2) < messanger_.stripe_interval;
			const bool y_in_stripe = y % (messanger_.stripe_interval * 2) < messanger_.stripe_interval;
			const bool do_create_map = x_in_stripe == y_in_stripe;

			if (do_create_map || x < (messanger_.map_start_rough_x - messanger_.map_min_x) / MapState::kMapPointDistance)
			{
				const float x_pos = messanger_.map_min_x + x * MapState::kMapPointDistance;	//ロボットの正面方向．
				const float y_pos = messanger_.map_min_y + y * MapState::kMapPointDistance;	//ロボットの側面方向．

				map->push_back({ x_pos, y_pos, messanger_.base_z });	//脚設置可能点を追加する．
			}
		}
	}
}

void SimulationMapCreator::CreateMeshMap(std::vector<dl::Vector3>* map) const
{
	assert(map != nullptr);		//mapがnullptrでないことを確認する．
	assert(map->empty());		//mapが空であることを確認する．

	// マップの xとyの存在範囲全体に脚設置可能点を敷き詰める．
	for (int x = 0; x < (messanger_.map_max_x - messanger_.map_min_x) / MapState::kMapPointDistance; x++)
	{
		for (int y = 0; y < (messanger_.map_max_y - messanger_.map_min_y) / MapState::kMapPointDistance; y++)
		{
			//網目模様をつくるために，一定間隔ごとに追加する．最初の待機場所の座標ならば無条件に追加する
			bool do_create_map;

			if ((x % (messanger_.stripe_interval * 2) < messanger_.stripe_interval))
			{
				if ((y % (messanger_.stripe_interval * 2) < messanger_.stripe_interval))
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

			if (do_create_map || x < (messanger_.map_start_rough_x - messanger_.map_min_x) / MapState::kMapPointDistance)
			{
				const float x_pos = messanger_.map_min_x + x * MapState::kMapPointDistance;	//ロボットの正面方向．
				const float y_pos = messanger_.map_min_y + y * MapState::kMapPointDistance;	//ロボットの側面方向．

				map->push_back({ x_pos, y_pos, messanger_.base_z });	//脚設置可能点を追加する．
			}
		}
	}
}

void SimulationMapCreator::CreateLatticePointMap(std::vector<dl::Vector3>* map) const
{
	assert(map != nullptr);		//mapがnullptrでないことを確認する．
	assert(map->empty());		//mapが空であることを確認する．

	// マップの xとyの存在範囲全体に脚設置可能点を敷き詰める．
	for (int x = 0; x < (messanger_.map_max_x - messanger_.map_min_x) / MapState::kMapPointDistance; x++)
	{
		for (int y = 0; y < (messanger_.map_max_y - messanger_.map_min_y) / MapState::kMapPointDistance; y++)
		{
			//網目模様をつくるために，一定間隔ごとに追加する．最初の待機場所の座標ならば無条件に追加する
			bool do_create_map;

			if ((x % (messanger_.stripe_interval * 2) < messanger_.stripe_interval))
			{
				if ((y % (messanger_.stripe_interval * 2) >= messanger_.stripe_interval))
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

			if (do_create_map || x < (messanger_.map_start_rough_x - messanger_.map_min_x) / MapState::kMapPointDistance)
			{
				const float x_pos = messanger_.map_min_x + x * MapState::kMapPointDistance;	//ロボットの正面方向．
				const float y_pos = messanger_.map_min_y + y * MapState::kMapPointDistance;	//ロボットの側面方向．

				map->push_back({ x_pos, y_pos, messanger_.base_z });	//脚設置可能点を追加する．
			}
		}
	}
}

void SimulationMapCreator::ChangeMapToPerforated(std::vector<dl::Vector3>* map) const
{
	assert(map != nullptr);		//mapがnullptrでないことを確認する．

	//厳密にホール率に合わせるために，まずはマップをSTRIPE_INTERVALにあわせて区切って，全部で何マスあるか調べる．
	const int cell_num_x = static_cast<int>((messanger_.map_start_rough_x - messanger_.map_min_x) / MapState::kMapPointDistance) / messanger_.stripe_interval;
	const int cell_num_y = static_cast<int>((messanger_.map_max_y - messanger_.map_min_y) / MapState::kMapPointDistance) / messanger_.stripe_interval;
	const int cell_sum = cell_num_x * cell_num_y;

	//マスの数だけ要素を持つvectorを用意する．値は全てfalseで初期化する．
	std::vector<bool> do_perforated(cell_sum, false);

	//ホール率に合わせて，値をtrueに変更する
	const int hole_num = cell_sum * messanger_.hole_rate / 100;

	for (int i = 0; i < hole_num; i++)
	{
		do_perforated.at(i) = true;
	}

	//ランダムなホールにするために要素の順番をシャッフルする．
	std::shuffle(std::begin(do_perforated), std::end(do_perforated), std::default_random_engine());

	//マップに穴をあける
	for (auto itr = (*map).begin(); itr != (*map).end();)
	{
		//待機場所の外に対してのみ作業をする
		if ((*itr).x < messanger_.map_start_rough_x)
		{
			itr++;
			continue;
		}

		//マスで区切るとどこに位置するかを調べる．
		const int cell_pos_x = static_cast<int>(((*itr).x - messanger_.map_start_rough_x) / MapState::kMapPointDistance) / messanger_.stripe_interval;
		const int cell_pos_y = static_cast<int>(((*itr).y - messanger_.map_min_y) / MapState::kMapPointDistance) / messanger_.stripe_interval;
		const int cell_index = cell_pos_x * cell_num_y + cell_pos_y;

		// cell_posの値がおかしくないかチェックする
		if (0 <= cell_index && cell_index < do_perforated.size())
		{
			//穴あけをする場所ならば
			if (do_perforated[cell_index])
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

void SimulationMapCreator::ChangeMapToStep(std::vector<dl::Vector3>* map) const
{
	assert(map != nullptr);		//mapがnullptrでないことを確認する．

	for (auto& i : *map)
	{
		//待機場所の外に対してのみ作業をする．
		if (i.x > messanger_.map_start_rough_x)
		{
			//階段の何段目かを計算する．待機場所のすぐ上が1段目なので1を足している．
			const int step_count = 1 + static_cast<int>((i.x - messanger_.map_start_rough_x) / messanger_.step_length);

			//階段状にZ座標を変更する．
			i.z += messanger_.step_height * step_count;
		}
	}
}

void SimulationMapCreator::ChangeMapToSlope(std::vector<dl::Vector3>* map) const
{
	assert(map != nullptr);		//mapがnullptrでないことを確認する．

	for (auto& i : *map)
	{
		//待機場所の外に対してのみ作業をする
		if (i.x > messanger_.map_start_rough_x)
		{
			//階段状にZ座標を変更する
			i.z += (i.x - messanger_.map_start_rough_x) * tan(messanger_.slope_angle);
		}
	}
}

void SimulationMapCreator::ChangeMapToTilt(std::vector<dl::Vector3>* map) const
{
	assert(map != nullptr);		//mapがnullptrでないことを確認する．

	for (auto& i : *map)
	{
		//待機場所の外に対してのみ作業をする
		if (i.x > messanger_.map_start_rough_x)
		{
			//階段状にZ座標を変更する
			i.z += i.y * tan(messanger_.tilt_angle);
		}
	}
}

void SimulationMapCreator::ChangeMapToRough(std::vector<dl::Vector3>* map) const
{
	assert(map != nullptr);		//mapがnullptrでないことを確認する．

	//まずはマップをSTRIPE_INTERVALにあわせて区切って，全部で何マスあるか調べる．
	const int cell_num_x = static_cast<int>((messanger_.map_start_rough_x - messanger_.map_min_x) / MapState::kMapPointDistance) / messanger_.stripe_interval;
	const int cell_num_y = static_cast<int>((messanger_.map_max_y - messanger_.map_min_y) / MapState::kMapPointDistance) / messanger_.stripe_interval;
	const int cell_sum = cell_num_x * cell_num_y;

	//マスの数だけ要素を持つvectorを用意する．
	std::vector<float> change_z_length;

	for (int i = 0; i < cell_sum; i++)
	{
		//ランダムなZ座標を入れる．
		change_z_length.push_back(dlm::GenerateRandomNumber(messanger_.routh_min_height, messanger_.routh_max_height));
	}

	for (auto& i : *map)
	{
		if (i.x > messanger_.map_start_rough_x)
		{
			//マスで区切るとどこに位置するかを調べる．
			const int cell_pos_x = static_cast<int>((i.x - messanger_.map_start_rough_x) / MapState::kMapPointDistance) / messanger_.stripe_interval;
			const int cell_pos_y = static_cast<int>((i.y - messanger_.map_min_y) / MapState::kMapPointDistance) / messanger_.stripe_interval;
			const int cell_index = cell_pos_x * cell_num_y + cell_pos_y;

			// cell_posの値がおかしくないかチェックする
			if (0 <= cell_index && cell_index < change_z_length.size())
			{
				i.z += change_z_length[cell_index];
			}
		}
	}
}