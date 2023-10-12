//! @file map_creator.h
//! @brief マップ生成クラス．


#ifndef DESIGNLAB_MAP_CREATOR_H_
#define DESIGNLAB_MAP_CREATOR_H_


#include <vector>

#include "designlab_vector3.h"
#include "map_state.h"


//! @enum MapCreateMode
//! @brief getMap関数のマップ生成のモードを指定する列挙体．
enum class MapCreateMode : int 
{
	kFlat = 0,			//!< 普通の平らな面を生成する．
	VERTICAL_STRIPE,	//!< 縦じまの面を生成する．
	HORIZONTAL_STRIPE,	//!< 横じまの面を生成する．
	DIAGONAL_STRIPE,	//!< 斜めじまの面を生成する．
	MESH,				//!< 格子状の面を生成する．網目状の地形ともいっていい．
	LATTICE_POINT,		//!< 格子点の面を生成する．網目状の逆
	READ_FROM_FILE		//!< ファイルから読み込む．失敗した場合普通の平らな面を生成する．
};


//! @enum MapCreateOption
//! @brief getMap関数のマップ生成のオプションを指定する列挙体．
//! @n bit演算を利用して複数指定できる．例えば穴あきかつ，階段状にしたいならば，MapCreateOption::kPerforated | MapCreateOption::kStep と指定する
//! @n bit演算ができるようにunsigned int型にしている．
enum class MapCreateOption : unsigned int 
{
	// 1 <<  x は 2^x を表す．

	kNone = 0,				//!< 特にオプションを指定しない．
	kPerforated = 1 << 0,	//!< 穴の空いた面を生成する．
	kStepe = 1 << 1,		//!< 階段状の地形を生成する．
	kSlope = 1 << 2,		//!< スロープ状の地形を生成する．
	kTilt = 1 << 3,			//!< 縦軸を中心軸として回転させた地形を生成する．
	Rough = 1 << 4,			//!< 凸凹の地形を生成する．
};

//! @class MapCreator
//! @brief マップを生成するクラス．
//! @details MapStateクラスの初期化処理のなかで呼んで，マップを生成してくれるクラス．
//! @n 先行研究のプログラムでは mainfunction.h で実装されていた処理である．
//! @n マップに関しては，仕様が定まっていないので，ひとまずこのまま
class MapCreator final
{
public:

	//! @brief マップ情報を与える関数 
	//! @n getMapのオプションはbit演算を利用して複数指定できる．例えば穴あきかつ，階段状にしたいならば，OPTION_PERFORATED | OPTION_STEP と指定する
	//! @param [in] mode 列挙体 MapCreateMode でどのような地形を生成するか指定する．
	//! @param [in] option メンバ変数の OPTION_????で生成するマップのオプションを指定する．
	MapState Create(MapCreateMode mode, MapCreateOption option);


private:

	//! フラットなマップを生成する
	void createFlatMap(std::vector<designlab::Vector3>* map);

	//! 縦じま模様の穴が開いたマップを生成する
	void createVerticalStripeMap(std::vector<designlab::Vector3>* map);

	//! 横じま模様の穴が開いたマップを生成する
	void createHorizontalStripeMap(std::vector<designlab::Vector3>* map);

	//! 斜めじま模様の穴が開いたマップを生成する
	void createDiagonalStripeMap(std::vector<designlab::Vector3>* map);

	//! 網目模様の穴が開いたマップを生成する
	void createMeshMap(std::vector<designlab::Vector3>* map);

	//! 網目模様の穴が開いたマップを生成する
	void createLatticePointMap(std::vector<designlab::Vector3>* map);

	//! ファイルからmapのデータを読みだして，それをもとにマップを作成する．
	bool createMapFromFile(std::vector<designlab::Vector3>* map);

	//生成されたマップにランダムな穴をあける
	void changeMapPerforated(std::vector<designlab::Vector3>* map);

	//生成されたマップを階段状にする
	void changeMapStep(std::vector<designlab::Vector3>* map);

	//生成されたマップを坂道にする
	void changeMapSlope(std::vector<designlab::Vector3>* map);

	//生成されたマップを傾ける
	void changeMapTilt(std::vector<designlab::Vector3>* map);

	//生成されたマップをデコボコにする
	void changeMapRough(std::vector<designlab::Vector3>* map);
};


#endif // !DESIGNLAB_MAP_CREATOR_H_