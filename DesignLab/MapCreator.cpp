#include "MapCreator.h"
#include "hexapod.h"
#include "MyMath.h"
#include <iostream>
#include <random>
#include <algorithm>
#include <string>
#include <sstream>

void MapCreator::create(const EMapCreateMode _mode, const int _option, std::vector<myvector::SVector>& _map_data, const bool _do_output)
{
	switch (_mode)
	{
	case EMapCreateMode::Flat:

		createFlatMap(_map_data);
		break;

	case EMapCreateMode::VerticalStripe:

		createVerticalStripeMap(_map_data);
		break;

	case EMapCreateMode::HorizontalStripe:

		createHorizontalStripeMap(_map_data);
		break;

	case EMapCreateMode::DiagonalStripe:

		createDiagonalStripeMap(_map_data);
		break;

	case EMapCreateMode::Mesh:

		createMeshMap(_map_data);
		break;

	case EMapCreateMode::LatticePoint:

		createLatticePointMap(_map_data);
		break;

	case EMapCreateMode::ReadFromFile:

		//ファイルからマップを生成する．
		if (createMapFromFile(_map_data) == false)
		{
			//読み込みに失敗したら，平面のマップを生成する．
			createFlatMap(_map_data);
		}
		break;

	default:

		//異常な値が入力されたら，平面のマップを生成する．
		createFlatMap(_map_data);
		break;
	}

	//オプション指定に基づき，Z座標を変更する
	if (_option & OPTION_PERFORATED && _mode != EMapCreateMode::ReadFromFile)
	{
		//穴あき地形にする．
		changeMapPerforated(_map_data);
	}

	if (_option & OPTION_STEP && _mode != EMapCreateMode::ReadFromFile)
	{
		//階段状にする．
		changeMapStep(_map_data);
	}

	if (_option & OPTION_SLOPE && _mode != EMapCreateMode::ReadFromFile)
	{
		//坂道にする．
		changeMapSlope(_map_data);
	}

	if (_option & OPTION_TILT && _mode != EMapCreateMode::ReadFromFile)
	{
		//坂道にする．
		changeMapTilt(_map_data);
	}

	if (_option & OPTION_ROUGH && _mode != EMapCreateMode::ReadFromFile)
	{
		//デコボコにする．
		changeMapRough(_map_data);
	}

	//ファイルへ出力する
	if (_do_output == true)
	{
		writeMapToFile(_map_data);
	}

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
}

void MapCreator::createFlatMap(std::vector<myvector::SVector> &_map)
{
	_map.clear();	//マップを初期化する.

	// マップの xとyの存在範囲全体に脚設置可能点を敷き詰める．
	for (int x = 0; x < (MapConst::MAP_MAX_FORWARD - MapConst::MAP_MIN_FORWARD) / MapConst::FOOT_HOLD_XY_DIST; x++)
	{
		for (int y = 0; y < (MapConst::MAP_MAX_HORIZONTAL - MapConst::MAP_MIN_HORIZONTAL) / MapConst::FOOT_HOLD_XY_DIST; y++)
		{
			const float _x = MapConst::MAP_MIN_FORWARD + (float)x * MapConst::FOOT_HOLD_XY_DIST;	//ロボットの正面方向．
			const float _y = MapConst::MAP_MIN_HORIZONTAL + (float)y * MapConst::FOOT_HOLD_XY_DIST;	//ロボットの側面方向．

			_map.push_back( myvector::SVector(_x, _y, MapConst::MAX_Z_BASE) );	//脚設置可能点を追加する．
		}
	}
}

void MapCreator::createVerticalStripeMap(std::vector<myvector::SVector>& _map)
{
	_map.clear();	//マップを初期化する.

	// マップの xとyの存在範囲全体に脚設置可能点を敷き詰める．
	for (int x = 0; x < (MapConst::MAP_MAX_FORWARD - MapConst::MAP_MIN_FORWARD) / MapConst::FOOT_HOLD_XY_DIST; x++)
	{
		for (int y = 0; y < (MapConst::MAP_MAX_HORIZONTAL - MapConst::MAP_MIN_HORIZONTAL) / MapConst::FOOT_HOLD_XY_DIST; y++)
		{
			//縦じまをつくるために，一定間隔ごとに追加する．最初の待機場所の座標ならば無条件に追加する
			if (y % (MapConst::STRIPE_INTERVAL * 2) < MapConst::STRIPE_INTERVAL || x < (MapConst::MAP_START_ROUGH - MapConst::MAP_MIN_FORWARD) / MapConst::FOOT_HOLD_XY_DIST)
			{
				const float _x = MapConst::MAP_MIN_FORWARD + (float)x * MapConst::FOOT_HOLD_XY_DIST;	//ロボットの正面方向．
				const float _y = MapConst::MAP_MIN_HORIZONTAL + (float)y * MapConst::FOOT_HOLD_XY_DIST;	//ロボットの側面方向．

				_map.push_back( myvector::SVector(_x, _y, MapConst::MAX_Z_BASE) );	//脚設置可能点を追加する．
			}
		}
	}
}

void MapCreator::createHorizontalStripeMap(std::vector<myvector::SVector>& _map)
{
	_map.clear();	//マップを初期化する.

	// マップの xとyの存在範囲全体に脚設置可能点を敷き詰める．
	for (int x = 0; x < (MapConst::MAP_MAX_FORWARD - MapConst::MAP_MIN_FORWARD) / MapConst::FOOT_HOLD_XY_DIST; x++)
	{
		for (int y = 0; y < (MapConst::MAP_MAX_HORIZONTAL - MapConst::MAP_MIN_HORIZONTAL) / MapConst::FOOT_HOLD_XY_DIST; y++)
		{
			//縦じまをつくるために，一定間隔ごとに追加する．最初の待機場所の座標ならば無条件に追加する
			if (x % (MapConst::STRIPE_INTERVAL * 2) < MapConst::STRIPE_INTERVAL || x < (MapConst::MAP_START_ROUGH - MapConst::MAP_MIN_FORWARD) / MapConst::FOOT_HOLD_XY_DIST)
			{
				const float _x = MapConst::MAP_MIN_FORWARD + (float)x * MapConst::FOOT_HOLD_XY_DIST;	//ロボットの正面方向．
				const float _y = MapConst::MAP_MIN_HORIZONTAL + (float)y * MapConst::FOOT_HOLD_XY_DIST;	//ロボットの側面方向．

				_map.push_back( myvector::SVector(_x, _y, MapConst::MAX_Z_BASE) );	//脚設置可能点を追加する．
			}
		}
	}
}

void MapCreator::createDiagonalStripeMap(std::vector<myvector::SVector>& _map)
{
	_map.clear();	//マップを初期化する.

	// マップの xとyの存在範囲全体に脚設置可能点を敷き詰める．
	for (int x = 0; x < (MapConst::MAP_MAX_FORWARD - MapConst::MAP_MIN_FORWARD) / MapConst::FOOT_HOLD_XY_DIST; x++)
	{
		for (int y = 0; y < (MapConst::MAP_MAX_HORIZONTAL - MapConst::MAP_MIN_HORIZONTAL) / MapConst::FOOT_HOLD_XY_DIST; y++)
		{
			//斜めじまをつくるために，一定間隔ごとに追加する．最初の待機場所の座標ならば無条件に追加する
			const bool _do_create_map = (y % (MapConst::STRIPE_INTERVAL * 2) < MapConst::STRIPE_INTERVAL) ^ (x % (MapConst::STRIPE_INTERVAL * 2) < MapConst::STRIPE_INTERVAL);

			if (_do_create_map || x < (MapConst::MAP_START_ROUGH - MapConst::MAP_MIN_FORWARD) / MapConst::FOOT_HOLD_XY_DIST)
			{
				const float _x = MapConst::MAP_MIN_FORWARD + (float)x * MapConst::FOOT_HOLD_XY_DIST;	//ロボットの正面方向．
				const float _y = MapConst::MAP_MIN_HORIZONTAL + (float)y * MapConst::FOOT_HOLD_XY_DIST;	//ロボットの側面方向．

				_map.push_back(myvector::SVector(_x, _y, MapConst::MAX_Z_BASE));	//脚設置可能点を追加する．
			}
		}
	}
}

void MapCreator::createMeshMap(std::vector<myvector::SVector>& _map)
{
	_map.clear();	//マップを初期化する.

	// マップの xとyの存在範囲全体に脚設置可能点を敷き詰める．
	for (int x = 0; x < (MapConst::MAP_MAX_FORWARD - MapConst::MAP_MIN_FORWARD) / MapConst::FOOT_HOLD_XY_DIST; x++)
	{
		for (int y = 0; y < (MapConst::MAP_MAX_HORIZONTAL - MapConst::MAP_MIN_HORIZONTAL) / MapConst::FOOT_HOLD_XY_DIST; y++)
		{
			//網目模様をつくるために，一定間隔ごとに追加する．最初の待機場所の座標ならば無条件に追加する
			bool _do_create_map;

			if ((x % (MapConst::STRIPE_INTERVAL * 2) < MapConst::STRIPE_INTERVAL)) 
			{
				if ((y % (MapConst::STRIPE_INTERVAL * 2) < MapConst::STRIPE_INTERVAL)) 
				{
					_do_create_map = true;
				}
				else 
				{
					_do_create_map = false;
				}
			}
			else 
			{
				_do_create_map = true;
			}

			if (_do_create_map || x < (MapConst::MAP_START_ROUGH - MapConst::MAP_MIN_FORWARD) / MapConst::FOOT_HOLD_XY_DIST)
			{
				const float _x = MapConst::MAP_MIN_FORWARD + (float)x * MapConst::FOOT_HOLD_XY_DIST;	//ロボットの正面方向．
				const float _y = MapConst::MAP_MIN_HORIZONTAL + (float)y * MapConst::FOOT_HOLD_XY_DIST;	//ロボットの側面方向．

				_map.push_back( myvector::SVector(_x, _y, MapConst::MAX_Z_BASE) );	//脚設置可能点を追加する．
			}
		}
	}
}

void MapCreator::createLatticePointMap(std::vector<myvector::SVector>& _map)
{
	_map.clear();	//マップを初期化する.

	// マップの xとyの存在範囲全体に脚設置可能点を敷き詰める．
	for (int x = 0; x < (MapConst::MAP_MAX_FORWARD - MapConst::MAP_MIN_FORWARD) / MapConst::FOOT_HOLD_XY_DIST; x++)
	{
		for (int y = 0; y < (MapConst::MAP_MAX_HORIZONTAL - MapConst::MAP_MIN_HORIZONTAL) / MapConst::FOOT_HOLD_XY_DIST; y++)
		{
			//網目模様をつくるために，一定間隔ごとに追加する．最初の待機場所の座標ならば無条件に追加する
			bool _do_create_map;

			if ((x % (MapConst::STRIPE_INTERVAL * 2) < MapConst::STRIPE_INTERVAL))
			{
				if ((y % (MapConst::STRIPE_INTERVAL * 2) >= MapConst::STRIPE_INTERVAL))
				{
					_do_create_map = false;
				}
				else
				{
					_do_create_map = true;
				}
			}
			else
			{
				_do_create_map = false;
			}

			if (_do_create_map || x < (MapConst::MAP_START_ROUGH - MapConst::MAP_MIN_FORWARD) / MapConst::FOOT_HOLD_XY_DIST)
			{
				const float _x = MapConst::MAP_MIN_FORWARD + (float)x * MapConst::FOOT_HOLD_XY_DIST;	//ロボットの正面方向．
				const float _y = MapConst::MAP_MIN_HORIZONTAL + (float)y * MapConst::FOOT_HOLD_XY_DIST;	//ロボットの側面方向．

				_map.push_back( myvector::SVector(_x, _y, MapConst::MAX_Z_BASE) );	//脚設置可能点を追加する．
			}
		}
	}
}

bool MapCreator::createMapFromFile(std::vector<myvector::SVector>& _map)
{
	//マップデータを初期化する
	_map.clear();

	std::ifstream ifs(MapConst::INPUT_FILE_NAME);//読み込むファイル
	std::string line;//ファイルの行

	//ファイルを開くことに失敗したら
	if (!ifs.is_open())
	{
		std::cout << "マップデータファイルが開けません．" << std::endl;
		return false;
	}

	// 1行目には数値がないため，読み込まない
	bool _is_first_line = true;

	// 1行ごとに読み込む
	while (std::getline(ifs, line))
	{
		std::istringstream stream(line);
		std::string field;
		std::vector<std::string> result;

		while (std::getline(stream, field, ','))
		{
			//行の読み込みが終わるまで
			result.push_back(field);//","ごとにresultにプッシュバック//この時点では文字として入力される
		}

		if (_is_first_line == false)
		{
			//stringをfloatに変換して代入
			myvector::SVector _pos(stof(result.at(1)), stof(result.at(2)), stof(result.at(3)));
			_map.push_back(_pos);
		}
		else { _is_first_line = false; }

	}

	ifs.close();

	return true;
}

void MapCreator::changeMapPerforated(std::vector<myvector::SVector>& _map)
{
	//厳密にホール率に合わせるために，まずはマップをSTRIPE_INTERVALにあわせて区切って，全部で何マスあるか調べる．
	const int _cell_num_x = (MapConst::MAP_MAX_FORWARD - MapConst::MAP_START_ROUGH) / MapConst::FOOT_HOLD_XY_DIST / MapConst::STRIPE_INTERVAL;
	const int _cell_num_y = (MapConst::MAP_MAX_HORIZONTAL - MapConst::MAP_MIN_HORIZONTAL) / MapConst::FOOT_HOLD_XY_DIST / MapConst::STRIPE_INTERVAL;
	const int _cell_num = _cell_num_x * _cell_num_y;

	//マスの数だけ要素を持つvectorを用意する．値は全てfalseで初期化する．
	std::vector<bool> _do_perforated(_cell_num, false);

	//ホール率に合わせて，値をtrueに変更する
	int _hole_num = _cell_num * MapConst::HOLE_RATE / 100;

	for (int i = 0; i < _hole_num; i++)
	{
		_do_perforated.at(i) = true;
	}

	//ランダムなホールにするために要素の順番をシャッフルする．
	std::shuffle(std::begin(_do_perforated), std::end(_do_perforated), std::default_random_engine());

	//マップに穴をあける
	for (auto itr = _map.begin(); itr != _map.end();)
	{
		//待機場所の外に対してのみ作業をする
		if ((*itr).x < MapConst::MAP_START_ROUGH)
		{
			itr++;
			continue;
		}

		//マスで区切るとどこに位置するかを調べる．
		const int _cell_pos_x = (int)((*itr).x - MapConst::MAP_START_ROUGH) / MapConst::FOOT_HOLD_XY_DIST / MapConst::STRIPE_INTERVAL;
		const int _cell_pos_y = (int)((*itr).y - MapConst::MAP_MIN_HORIZONTAL) / MapConst::FOOT_HOLD_XY_DIST / MapConst::STRIPE_INTERVAL;
		const int _cell_pos = _cell_pos_x * _cell_num_y + _cell_pos_y;

		// cell_posの値がおかしくないかチェックする
		if (0 <= _cell_pos && _cell_pos < _do_perforated.size())
		{
			//穴あけをする場所ならば
			if (_do_perforated.at(_cell_pos) == true)
			{
				//脚設置可能点を消してイテレータを更新する
				itr = _map.erase(itr);
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

void MapCreator::changeMapStep(std::vector<myvector::SVector>& _map)
{
	for (auto& i : _map)
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

void MapCreator::changeMapSlope(std::vector<myvector::SVector>& _map)
{
	for (auto& i : _map)
	{
		//待機場所の外に対してのみ作業をする
		if (i.x > MapConst::MAP_START_ROUGH)
		{
			//階段状にZ座標を変更する
			i.z += (i.x - MapConst::MAP_START_ROUGH) * tan(MapConst::SLOPE_ANGLE / 180 * Define::MY_PI);
		}
	}
}

void MapCreator::changeMapTilt(std::vector<myvector::SVector>& _map)
{
	for (auto& i : _map)
	{
		//待機場所の外に対してのみ作業をする
		if (i.x > MapConst::MAP_START_ROUGH)
		{
			//階段状にZ座標を変更する
			i.z += i.y * tan(MapConst::TILT_ANGLE / 180 * Define::MY_PI);
		}
	}
}

void MapCreator::changeMapRough(std::vector<myvector::SVector>& _map)
{
	//まずはマップをSTRIPE_INTERVALにあわせて区切って，全部で何マスあるか調べる．
	const int _cell_num_x = (MapConst::MAP_MAX_FORWARD - MapConst::MAP_START_ROUGH) / MapConst::FOOT_HOLD_XY_DIST / MapConst::STRIPE_INTERVAL;
	const int _cell_num_y = (MapConst::MAP_MAX_HORIZONTAL - MapConst::MAP_MIN_HORIZONTAL) / MapConst::FOOT_HOLD_XY_DIST / MapConst::STRIPE_INTERVAL;
	const int _cell_num = _cell_num_x * _cell_num_y;

	//マスの数だけ要素を持つvectorを用意する．
	std::vector<float> _change_z_lenght;

	for (int i = 0; i < _cell_num; i++)
	{
		//ランダムなZ座標を入れる．
		_change_z_lenght.push_back(my_math::generateRandomNumber(MapConst::ROUGH_MIN_HEIGHT, MapConst::ROUGH_MAX_HEIGHT));
	}

	for (auto& i : _map)
	{
		//マスで区切るとどこに位置するかを調べる．
		const int _cell_pos_x = (int)(i.x - MapConst::MAP_START_ROUGH) / MapConst::FOOT_HOLD_XY_DIST / MapConst::STRIPE_INTERVAL;
		const int _cell_pos_y = (int)(i.y - MapConst::MAP_MIN_HORIZONTAL) / MapConst::FOOT_HOLD_XY_DIST / MapConst::STRIPE_INTERVAL;
		const int _cell_pos = _cell_pos_x * _cell_num_y + _cell_pos_y;

		// cell_posの値がおかしくないかチェックする
		if (0 <= _cell_pos && _cell_pos < _change_z_lenght.size())
		{
			i.z += _change_z_lenght.at(_cell_pos);
		}
	}
}

bool MapCreator::writeMapToFile(const std::vector<myvector::SVector>& _map)
{
	std::ofstream _write_mapdata_file;

	_write_mapdata_file.open(MapConst::OUTPUT_FILE_NAME);

	if (_write_mapdata_file.is_open() == false)
	{
		//ファイルを開けなかったらfalse
		return false;
	}

	//ここからファイルに書き込む
	_write_mapdata_file << "n" << "," << "x" << "," << "y" << "," << "z" << std::endl;

	for (int i = 0; i < _map.size(); ++i)
	{
		_write_mapdata_file << i << "," << _map[i].x << "," << _map[i].y << "," << _map[i].z << "\n";
	}

	//ファイルを閉じる
	_write_mapdata_file.close();

	return true;
}
