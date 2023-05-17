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
	Mesh			= 5,	//網目状の面を生成する．
	LatticePoint	= 6,	//格子状の面を生成する．網目状の逆
	ReadFromFile	= 7		//ファイルから読み込む．失敗した場合普通の平らな面を生成する．
};

//このクラスは所謂ユーティリティクラスです．実体は生成できないので注意．
class MapCreator final
{
public:

	//マップ情報を与える関数
	static void getMap(const EMapCreateMode _mode, const int _option, myvector::SVector p_mapData3D[MapConst::MAPDATA3D_MAX], int* mapData3D_MAX, LNODE* CurrentCondition, const int f);

	//getMapのオプションはbit演算を利用して複数指定できる．例えば穴あきかつ，階段状にしたいならば
	// OPTION_PERFORATED | OPTION_STEP と指定する

	constexpr static int OPTION_NONE		= 0b0000;	//特にオプションを指定しない．
	constexpr static int OPTION_PERFORATED	= 0b0001;	//穴の空いた面を生成する．
	constexpr static int OPTION_STEP		= 0b0010;	//階段状の地形を生成する．
	constexpr static int OPTION_SLOPE		= 0b0100;	//スロープ状の地形を生成する．
	constexpr static int OPTION_ROUGH		= 0b1000;	//凸凹の地形を生成する．

private:
	//実体は利用しないので，コンストラクタを消去
	MapCreator() = delete;
	MapCreator(const MapCreator& _other) = delete;


	//現在のmapのデータファイルに書き出す．
	static void writeMapDataToFile(std::ofstream& filename, const myvector::SVector* map, const int* mapData3D_MAX);

	//フラットなマップを生成する
	static void createFlatMap(std::vector<myvector::SVector> &_map);

	//縦じま模様の穴が開いたマップを生成する
	static void createVerticalStripeMap(std::vector<myvector::SVector>& _map);

	//横じま模様の穴が開いたマップを生成する
	static void createHorizontalStripeMap(std::vector<myvector::SVector>& _map);

	//ファイルからmapのデータを読みだして，それをもとにマップを作成する．
	static bool createMapFromFile(std::vector<myvector::SVector>& _map);
};

//マップの平行移動
void recalMap(myvector::SVector *p_mapData3D, int mapData3D_MAX, LNODE *CurrentCondition, LNODE *PastCondition);

void MapSqrtDivide(myvector::SVector* mapData, int mapDataNum, std::vector< std::vector< std::vector<myvector::SVector> > >& divideMapData, int pointNum[MapConst::LP_DIVIDE_NUM][MapConst::LP_DIVIDE_NUM]);

void AreaDivide(myvector::SVector p1, myvector::SVector p2, int &x1, int &x2, int &y1, int &y2);

void SetConditionForStripe(LNODE& node, const int f);

//マップデータの数、乱数の最小値、乱数の最大値、乱数を格納する配列[マップデータの数]
void GetRandom(int mapDataNum, int min, int max, int* Random);