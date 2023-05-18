#include "MapCreator.h"
#include "hexapod.h"
#include "MyMath.h"
#include <iostream>
#include <random>
#include <algorithm>
#include <string>
#include <sstream>

void map_creator::createMap(const EMapCreateMode _mode, const int _option, myvector::SVector p_mapData3D[MapConst::MAPDATA3D_MAX], const LNODE& _current_condition, const bool _do_output)
{
	std::vector<myvector::SVector> _map_data;	//mapデータとは脚設置可能点の集合体のことである．面ではなく点のデータである．

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

	//空中から始まってしまうと困るので，ロボットの初期座標の足元に足場を設置する．
	Hexapod phantomX;
	phantomX.setMyPosition(_current_condition.global_center_of_mass);//重心位置グローバル
	phantomX.setMyDirection(_current_condition.pitch, _current_condition.roll, _current_condition.yaw);//姿勢(テイトブライアン角)グローバル、初期姿勢が違うときは変更する必要あり//(0,0,0)から変更
	phantomX.setLocalLegPos(_current_condition.Leg);
	phantomX.setLocalLeg2Pos(_current_condition.Leg2);//付け根から脚先の位置ローカル

	//Hexapodの初期脚座標を出力する
	for (int i = 0; i < HexapodConst::LEG_NUM; i++) { myvector::VectorOutPut(phantomX.getGlobalLegPos(i)); }
	for (int i = 0; i < HexapodConst::LEG_NUM; i++) { myvector::VectorOutPut(phantomX.getGlobalLeg2Pos(i)); }

	//初期姿勢における足先座標を脚設置可能点とする
	for (int i = 0; i < HexapodConst::LEG_NUM; i++)
	{
		_map_data.at(i) = phantomX.getGlobalLegPos(i);
	}

	//高すぎる地点にあるならば遠いところに飛ばす
	for (int i = 0; i < HexapodConst::LEG_NUM; i++)
	{
		if (_map_data.at(i).z >= 25)
		{
			_map_data.at(i).x = 10000;
		}
	}

	//結果を代入する
	for (int i = 0; i < MapConst::MAPDATA3D_MAX; i++)
	{
		//足りないならば，適当な座標へ飛ばす
		if (_map_data.size() <= i) 
		{
			myvector::SVector _pos(MapConst::INVALID_FOOT_HOLD, MapConst::INVALID_FOOT_HOLD, MapConst::INVALID_FOOT_HOLD);
			p_mapData3D[i] = _pos;
		}
		else 
		{
			p_mapData3D[i] = _map_data.at(i); 
		}
	}

	return;

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

void map_creator::createFlatMap(std::vector<myvector::SVector> &_map)
{
	_map.clear();	//マップを初期化する.

	// マップの xとyの存在範囲全体に脚設置可能点を敷き詰める．
	for (int x = 0; x < (MapConst::MAP_X_MAX - MapConst::MAP_X_MIN) / MapConst::FOOT_HOLD_XY_DIST; x++)
	{
		for (int y = 0; y < (MapConst::MAP_Y_MAX - MapConst::MAP_Y_MIN) / MapConst::FOOT_HOLD_XY_DIST; y++)
		{
			const double _x = MapConst::MAP_X_MIN + (double)x * MapConst::FOOT_HOLD_XY_DIST;	//ロボットに対して横向きの座標
			const double _y = MapConst::MAP_Y_MIN + (double)y * MapConst::FOOT_HOLD_XY_DIST;	//ロボットに対して横向きの座標

			myvector::SVector _pos(_x, _y, MapConst::MAX_Z_BASE);
			_map.push_back(_pos);
		}
	}
}

void map_creator::createVerticalStripeMap(std::vector<myvector::SVector>& _map)
{
	_map.clear();	//マップを初期化する.

	// マップの xとyの存在範囲全体に脚設置可能点を敷き詰める．
	for (int x = 0; x < (MapConst::MAP_X_MAX - MapConst::MAP_X_MIN) / MapConst::FOOT_HOLD_XY_DIST; x++)
	{
		for (int y = 0; y < (MapConst::MAP_Y_MAX - MapConst::MAP_Y_MIN) / MapConst::FOOT_HOLD_XY_DIST; y++)
		{
			//縦じまをつくるために，一定間隔ごとに追加する．最初の待機場所の座標ならば無条件に追加する
			if (x % (MapConst::STRIPE_INTERVAL * 2) < MapConst::STRIPE_INTERVAL || y < (MapConst::START_ROUGH_TARRAIN_Y - MapConst::MAP_Y_MIN) / MapConst::FOOT_HOLD_XY_DIST)
			{
				const double _x = MapConst::MAP_X_MIN + (double)x * MapConst::FOOT_HOLD_XY_DIST;	//ロボットに対して横向きの座標
				const double _y = MapConst::MAP_Y_MIN + (double)y * MapConst::FOOT_HOLD_XY_DIST;	//ロボットに対して横向きの座標

				myvector::SVector _pos(_x, _y, MapConst::MAX_Z_BASE);
				_map.push_back(_pos);
			}
		}
	}
}

void map_creator::createHorizontalStripeMap(std::vector<myvector::SVector>& _map)
{
	_map.clear();	//マップを初期化する.

	// マップの xとyの存在範囲全体に脚設置可能点を敷き詰める．
	for (int x = 0; x < (MapConst::MAP_X_MAX - MapConst::MAP_X_MIN) / MapConst::FOOT_HOLD_XY_DIST; x++)
	{
		for (int y = 0; y < (MapConst::MAP_Y_MAX - MapConst::MAP_Y_MIN) / MapConst::FOOT_HOLD_XY_DIST; y++)
		{
			//縦じまをつくるために，一定間隔ごとに追加する．最初の待機場所の座標ならば無条件に追加する
			if (y % (MapConst::STRIPE_INTERVAL * 2) < MapConst::STRIPE_INTERVAL || y < (MapConst::START_ROUGH_TARRAIN_Y - MapConst::MAP_Y_MIN) / MapConst::FOOT_HOLD_XY_DIST)
			{
				const double _x = MapConst::MAP_X_MIN + (double)x * MapConst::FOOT_HOLD_XY_DIST;	//ロボットに対して横向きの座標
				const double _y = MapConst::MAP_Y_MIN + (double)y * MapConst::FOOT_HOLD_XY_DIST;	//ロボットに対して横向きの座標

				myvector::SVector _pos(_x, _y, MapConst::MAX_Z_BASE);
				_map.push_back(_pos);
			}
		}
	}
}

void map_creator::createDiagonalStripeMap(std::vector<myvector::SVector>& _map)
{
	_map.clear();	//マップを初期化する.

	// マップの xとyの存在範囲全体に脚設置可能点を敷き詰める．
	for (int x = 0; x < (MapConst::MAP_X_MAX - MapConst::MAP_X_MIN) / MapConst::FOOT_HOLD_XY_DIST; x++)
	{
		for (int y = 0; y < (MapConst::MAP_Y_MAX - MapConst::MAP_Y_MIN) / MapConst::FOOT_HOLD_XY_DIST; y++)
		{
			//斜めじまをつくるために，一定間隔ごとに追加する．最初の待機場所の座標ならば無条件に追加する
			const bool _do_create_map = (x % (MapConst::STRIPE_INTERVAL * 2) < MapConst::STRIPE_INTERVAL) ^ (y % (MapConst::STRIPE_INTERVAL * 2) < MapConst::STRIPE_INTERVAL);

			if (_do_create_map || y < (MapConst::START_ROUGH_TARRAIN_Y - MapConst::MAP_Y_MIN) / MapConst::FOOT_HOLD_XY_DIST)
			{
				const double _x = MapConst::MAP_X_MIN + (double)x * MapConst::FOOT_HOLD_XY_DIST;	//ロボットに対して横向きの座標
				const double _y = MapConst::MAP_Y_MIN + (double)y * MapConst::FOOT_HOLD_XY_DIST;	//ロボットに対して横向きの座標

				myvector::SVector _pos(_x, _y, MapConst::MAX_Z_BASE);
				_map.push_back(_pos);
			}
		}
	}
}

void map_creator::createMeshMap(std::vector<myvector::SVector>& _map)
{
	_map.clear();	//マップを初期化する.

	// マップの xとyの存在範囲全体に脚設置可能点を敷き詰める．
	for (int x = 0; x < (MapConst::MAP_X_MAX - MapConst::MAP_X_MIN) / MapConst::FOOT_HOLD_XY_DIST; x++)
	{
		for (int y = 0; y < (MapConst::MAP_Y_MAX - MapConst::MAP_Y_MIN) / MapConst::FOOT_HOLD_XY_DIST; y++)
		{
			//網目模様をつくるために，一定間隔ごとに追加する．最初の待機場所の座標ならば無条件に追加する
			bool _do_create_map;

			if ((y % (MapConst::STRIPE_INTERVAL * 2) < MapConst::STRIPE_INTERVAL)) 
			{
				if ((x % (MapConst::STRIPE_INTERVAL * 2) < MapConst::STRIPE_INTERVAL)) 
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

			if (_do_create_map || y < (MapConst::START_ROUGH_TARRAIN_Y - MapConst::MAP_Y_MIN) / MapConst::FOOT_HOLD_XY_DIST)
			{
				const double _x = MapConst::MAP_X_MIN + (double)x * MapConst::FOOT_HOLD_XY_DIST;	//ロボットに対して横向きの座標
				const double _y = MapConst::MAP_Y_MIN + (double)y * MapConst::FOOT_HOLD_XY_DIST;	//ロボットに対して横向きの座標

				myvector::SVector _pos(_x, _y, MapConst::MAX_Z_BASE);
				_map.push_back(_pos);
			}
		}
	}
}

void map_creator::createLatticePointMap(std::vector<myvector::SVector>& _map)
{
	_map.clear();	//マップを初期化する.

	// マップの xとyの存在範囲全体に脚設置可能点を敷き詰める．
	for (int x = 0; x < (MapConst::MAP_X_MAX - MapConst::MAP_X_MIN) / MapConst::FOOT_HOLD_XY_DIST; x++)
	{
		for (int y = 0; y < (MapConst::MAP_Y_MAX - MapConst::MAP_Y_MIN) / MapConst::FOOT_HOLD_XY_DIST; y++)
		{
			//網目模様をつくるために，一定間隔ごとに追加する．最初の待機場所の座標ならば無条件に追加する
			bool _do_create_map;

			if ((y % (MapConst::STRIPE_INTERVAL * 2) < MapConst::STRIPE_INTERVAL))
			{
				if ((x % (MapConst::STRIPE_INTERVAL * 2) >= MapConst::STRIPE_INTERVAL))
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

			if (_do_create_map || y < (MapConst::START_ROUGH_TARRAIN_Y - MapConst::MAP_Y_MIN) / MapConst::FOOT_HOLD_XY_DIST)
			{
				const double _x = MapConst::MAP_X_MIN + (double)x * MapConst::FOOT_HOLD_XY_DIST;	//ロボットに対して横向きの座標
				const double _y = MapConst::MAP_Y_MIN + (double)y * MapConst::FOOT_HOLD_XY_DIST;	//ロボットに対して横向きの座標

				myvector::SVector _pos(_x, _y, MapConst::MAX_Z_BASE);
				_map.push_back(_pos);
			}
		}
	}
}

bool map_creator::createMapFromFile(std::vector<myvector::SVector>& _map)
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
			//stringをdoubleに変換して代入
			myvector::SVector _pos(stod(result.at(1)), stod(result.at(2)), stod(result.at(3)));
			_map.push_back(_pos);
		}
		else { _is_first_line = false; }

	}

	ifs.close();

	return true;
}

void map_creator::changeMapPerforated(std::vector<myvector::SVector>& _map)
{
	//厳密にホール率に合わせるために，まずはマップをSTRIPE_INTERVALにあわせて区切って，全部で何マスあるか調べる．
	const int _cell_num_x = (MapConst::MAP_X_MAX - MapConst::MAP_X_MIN) / MapConst::FOOT_HOLD_XY_DIST / MapConst::STRIPE_INTERVAL;
	const int _cell_num_y = (MapConst::MAP_Y_MAX - MapConst::START_ROUGH_TARRAIN_Y) / MapConst::FOOT_HOLD_XY_DIST / MapConst::STRIPE_INTERVAL;
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
		if ((*itr).y < MapConst::START_ROUGH_TARRAIN_Y)
		{
			itr++;
			continue;
		}

		//マスで区切るとどこに位置するかを調べる．
		const int _cell_pos_x = (int)((*itr).x - MapConst::MAP_X_MIN) / MapConst::FOOT_HOLD_XY_DIST / MapConst::STRIPE_INTERVAL;
		const int _cell_pos_y = (int)((*itr).y - MapConst::START_ROUGH_TARRAIN_Y) / MapConst::FOOT_HOLD_XY_DIST / MapConst::STRIPE_INTERVAL;
		const int _cell_pos = _cell_pos_y * _cell_num_x + _cell_pos_x;

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

void map_creator::changeMapStep(std::vector<myvector::SVector>& _map)
{
	for (auto& i : _map)
	{
		//待機場所の外に対してのみ作業をする
		if (i.y > MapConst::START_ROUGH_TARRAIN_Y) 
		{
			//階段の何段目かを計算する．待機場所のすぐ上が1段目なので1を足している
			const int _step_count = 1 + (int)((i.y - MapConst::START_ROUGH_TARRAIN_Y) / MapConst::STEP_LENGTH);

			//階段状にZ座標を変更する
			i.z += (double)MapConst::STEP_HEIGHT * _step_count;
		}
	}
}

void map_creator::changeMapSlope(std::vector<myvector::SVector>& _map)
{
	for (auto& i : _map)
	{
		//待機場所の外に対してのみ作業をする
		if (i.y > MapConst::START_ROUGH_TARRAIN_Y)
		{
			//階段状にZ座標を変更する
			i.z += (i.y - MapConst::START_ROUGH_TARRAIN_Y) * tan(MapConst::SLOPE_ANGLE / 180 * Define::MY_PI);
		}
	}
}

void map_creator::changeMapTilt(std::vector<myvector::SVector>& _map)
{
	for (auto& i : _map)
	{
		//待機場所の外に対してのみ作業をする
		if (i.y > MapConst::START_ROUGH_TARRAIN_Y)
		{
			//階段状にZ座標を変更する
			i.z += i.x * tan(MapConst::TILT_ANGLE / 180 * Define::MY_PI);
		}
	}
}

void map_creator::changeMapRough(std::vector<myvector::SVector>& _map)
{
	//まずはマップをSTRIPE_INTERVALにあわせて区切って，全部で何マスあるか調べる．
	const int _cell_num_x = (MapConst::MAP_X_MAX - MapConst::MAP_X_MIN) / MapConst::FOOT_HOLD_XY_DIST / MapConst::STRIPE_INTERVAL;
	const int _cell_num_y = (MapConst::MAP_Y_MAX - MapConst::START_ROUGH_TARRAIN_Y) / MapConst::FOOT_HOLD_XY_DIST / MapConst::STRIPE_INTERVAL;
	const int _cell_num = _cell_num_x * _cell_num_y;

	//マスの数だけ要素を持つvectorを用意する．
	std::vector<double> _change_z_lenght;

	for (int i = 0; i < _cell_num; i++)
	{
		//ランダムなZ座標を入れる．
		_change_z_lenght.push_back(my_math::generateRandomNumber(MapConst::ROUGH_MIN_HEIGHT, MapConst::ROUGH_MAX_HEIGHT));
	}

	for (auto& i : _map)
	{
		//マスで区切るとどこに位置するかを調べる．
		const int _cell_pos_x = (int)(i.x - MapConst::MAP_X_MIN) / MapConst::FOOT_HOLD_XY_DIST / MapConst::STRIPE_INTERVAL;
		const int _cell_pos_y = (int)(i.y - MapConst::START_ROUGH_TARRAIN_Y) / MapConst::FOOT_HOLD_XY_DIST / MapConst::STRIPE_INTERVAL;
		const int _cell_pos = _cell_pos_y * _cell_num_x + _cell_pos_x;

		// cell_posの値がおかしくないかチェックする
		if (0 <= _cell_pos && _cell_pos < _change_z_lenght.size())
		{
			i.z += _change_z_lenght.at(_cell_pos);
		}
	}
}

bool map_creator::writeMapToFile(const std::vector<myvector::SVector>& _map)
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
