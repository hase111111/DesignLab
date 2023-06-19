#pragma once
#include "MyVector.h"
#include <vector>


enum class EMapCreateMode : int
{
	Flat			= 1,	//!< 普通の平らな面を生成する．
	VerticalStripe	= 2,	//!< 縦じまの面を生成する．
	HorizontalStripe= 3,	//!< 横じまの面を生成する．
	DiagonalStripe	= 4,	//!< 斜めじまの面を生成する．
	Mesh			= 5,	//!< 格子状の面を生成する．網目状の地形ともいっていい．
	LatticePoint	= 6,	//!< 格子点の面を生成する．網目状の逆
	ReadFromFile	= 7		//!< ファイルから読み込む．失敗した場合普通の平らな面を生成する．
};


class MapCreator
{
public:

	//! マップ情報を与える関数 <br>
	//! getMapのオプションはbit演算を利用して複数指定できる．例えば穴あきかつ，階段状にしたいならば，OPTION_PERFORATED | OPTION_STEP と指定する
	//! @param [in] _mode 列挙体 EMapCreateModeでどのような地形を生成するか指定する．
	//! @param [in] _option メンバ変数の OPTION_????で生成するマップのオプションを指定する．_modeでReadFromFileが指定されているなら無視される．
	//! @param [out] _map_data 生成したマップはこの変数で参照しする．
	//! @param [in] _do_output 生成したマップをファイル出力するならtrue，しないならばfalse．する場合は生成した瞬間にファイル出力される．
	void create(const EMapCreateMode _mode, const int _option, std::vector<myvector::SVector>& _map_data, const bool _do_output);


	constexpr static int OPTION_NONE		= 0b00000;	//!< マップ生成のオプション：特にオプションを指定しない．
	constexpr static int OPTION_PERFORATED	= 0b00001;	//!< マップ生成のオプション：穴の空いた面を生成する．
	constexpr static int OPTION_STEP		= 0b00010;	//!< マップ生成のオプション：階段状の地形を生成する．
	constexpr static int OPTION_SLOPE		= 0b00100;	//!< マップ生成のオプション：スロープ状の地形を生成する．
	constexpr static int OPTION_TILT		= 0b01000;	//!< マップ生成のオプション：縦軸を中心軸として回転させた地形を生成する．
	constexpr static int OPTION_ROUGH		= 0b10000;	//!< マップ生成のオプション：凸凹の地形を生成する．

private:

	//フラットなマップを生成する
	void createFlatMap(std::vector<myvector::SVector> &_map);

	//縦じま模様の穴が開いたマップを生成する
	void createVerticalStripeMap(std::vector<myvector::SVector>& _map);

	//横じま模様の穴が開いたマップを生成する
	void createHorizontalStripeMap(std::vector<myvector::SVector>& _map);

	//斜めじま模様の穴が開いたマップを生成する
	void createDiagonalStripeMap(std::vector<myvector::SVector>& _map);

	//網目模様の穴が開いたマップを生成する
	void createMeshMap(std::vector<myvector::SVector>& _map);

	//網目模様の穴が開いたマップを生成する
	void createLatticePointMap(std::vector<myvector::SVector>& _map);

	//ファイルからmapのデータを読みだして，それをもとにマップを作成する．
	bool createMapFromFile(std::vector<myvector::SVector>& _map);

	//生成されたマップにランダムな穴をあける
	void changeMapPerforated(std::vector<myvector::SVector>& _map);

	//生成されたマップを階段状にする
	void changeMapStep(std::vector<myvector::SVector>& _map);

	//生成されたマップを坂道にする
	void changeMapSlope(std::vector<myvector::SVector>& _map);

	//生成されたマップを傾ける
	void changeMapTilt(std::vector<myvector::SVector>& _map);

	//生成されたマップをデコボコにする
	void changeMapRough(std::vector<myvector::SVector>& _map);

	//現在のmapのデータファイルに書き出す．
	bool writeMapToFile(const std::vector<myvector::SVector>& _map);
};


//! @file MapCreator.h
//! @brief マップ生成クラスの実装．マップ生成のモードを指定する列挙体の実装．
//! @author 長谷川

//! @enum EMapCreateMode
//! @brief getMap関数のマップ生成のモードを指定する列挙体．
//! @author 長谷川
 
//! @class MapCreator
//! @brief マップを生成するクラス．
//! @details MapStateクラスの初期化処理のなかで呼んで，マップを生成してくれるクラス．<br>
//! 先行研究のプログラムでは mainfunction.h で実装されていた処理である．
//! @author 長谷川
