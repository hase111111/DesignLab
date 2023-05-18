#pragma once
#include "vectorFunc.h"
#include "listFunc.h"
#include "MapConst.h"
#include <vector>
//#include "mapData.h"

// 旧名 mainfunction.h なにをしているのか分かりづらかったので，MapCreator.h にリネームしました．

//getMap関数のマップ生成のモードを指定する
enum class EMapCreateMode : int
{
	Flat			= 1,	//普通の平らな面を生成する．
	VerticalStripe	= 2,	//縦じまの面を生成する．
	HorizontalStripe= 3,	//横じまの面を生成する．
	DiagonalStripe	= 4,	//斜めじまの面を生成する．
	Mesh			= 5,	//格子状の面を生成する．網目状の地形ともいっていい．
	LatticePoint	= 6,	//格子点の面を生成する．網目状の逆
	ReadFromFile	= 7		//ファイルから読み込む．失敗した場合普通の平らな面を生成する．
};


//このクラスは所謂ユーティリティクラスです．実体は生成できないので注意．
class MapCreator final
{
public:

	//マップ情報を与える関数
	static void createMap(const EMapCreateMode _mode, const int _option, myvector::SVector p_mapData3D[MapConst::MAPDATA3D_MAX], const LNODE& _current_condition, const bool _do_output);

	//getMapのオプションはbit演算を利用して複数指定できる．例えば穴あきかつ，階段状にしたいならば
	// OPTION_PERFORATED | OPTION_STEP と指定する

	constexpr static int OPTION_NONE		= 0b00000;	//特にオプションを指定しない．
	constexpr static int OPTION_PERFORATED	= 0b00001;	//穴の空いた面を生成する．
	constexpr static int OPTION_STEP		= 0b00010;	//階段状の地形を生成する．
	constexpr static int OPTION_SLOPE		= 0b00100;	//スロープ状の地形を生成する．
	constexpr static int OPTION_TILT		= 0b01000;	//縦軸を中心軸として回転させた地形を生成する．
	constexpr static int OPTION_ROUGH		= 0b10000;	//凸凹の地形を生成する．

private:
	//実体は利用しないので，コンストラクタを消去
	MapCreator() = delete;
	MapCreator(const MapCreator& _other) = delete;


	//フラットなマップを生成する
	static void createFlatMap(std::vector<myvector::SVector> &_map);

	//縦じま模様の穴が開いたマップを生成する
	static void createVerticalStripeMap(std::vector<myvector::SVector>& _map);

	//横じま模様の穴が開いたマップを生成する
	static void createHorizontalStripeMap(std::vector<myvector::SVector>& _map);

	//斜めじま模様の穴が開いたマップを生成する
	static void createDiagonalStripeMap(std::vector<myvector::SVector>& _map);

	//網目模様の穴が開いたマップを生成する
	static void createMeshMap(std::vector<myvector::SVector>& _map);

	//網目模様の穴が開いたマップを生成する
	static void createLatticePointMap(std::vector<myvector::SVector>& _map);

	//ファイルからmapのデータを読みだして，それをもとにマップを作成する．
	static bool createMapFromFile(std::vector<myvector::SVector>& _map);


	//生成されたマップにランダムな穴をあける
	static void changeMapPerforated(std::vector<myvector::SVector>& _map);

	//生成されたマップを階段状にする
	static void changeMapStep(std::vector<myvector::SVector>& _map);

	//生成されたマップを坂道にする
	static void changeMapSlope(std::vector<myvector::SVector>& _map);

	//生成されたマップを傾ける
	static void changeMapTilt(std::vector<myvector::SVector>& _map);

	//生成されたマップをデコボコにする
	static void changeMapRough(std::vector<myvector::SVector>& _map);


	//現在のmapのデータファイルに書き出す．
	static bool writeMapToFile(const std::vector<myvector::SVector>& _map);
};
