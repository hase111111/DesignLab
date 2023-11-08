//! @file map_creator.h
//! @brief マップ生成クラス．


#ifndef DESIGNLAB_MAP_CREATOR_H_
#define DESIGNLAB_MAP_CREATOR_H_


#include <vector>

#include "designlab_vector3.h"
#include "interface_map_creator.h"
#include "map_state.h"


//! @enum MapCreateMode
//! @brief getMap関数のマップ生成のモードを指定する列挙体．
enum class MapCreateMode : int 
{
	kFlat = 0,			//!< 普通の平らな面を生成する．
	kVerticalStripe,	//!< 縦じまの面を生成する．
	kHorizontalStripe,	//!< 横じまの面を生成する．
	kDiagonalStripe,	//!< 斜めじまの面を生成する．
	kMesh,				//!< 格子状の面を生成する．網目状の地形ともいっていい．
	kLatticePoint,		//!< 格子点の面を生成する．網目状の逆
};


//! @enum MapCreateOption
//! @brief getMap関数のマップ生成のオプションを指定する列挙体．
//! @n bit演算を利用して複数指定できる．例えば穴あきかつ，階段状にしたいならば，MapCreateOption::kPerforated | MapCreateOption::kStep と指定する
//! @n bit演算ができるようにunsigned int型にしている．
enum class MapCreateOption : unsigned int 
{
	// 1 <<  x は 2^x を表す．

	kNone = 0,				//!< 特にオプションを指定しない．
	kPerforated = 1 << 0,	//!< 穴の空いたマップに変化させる．
	kStep = 1 << 1,			//!< 階段状の地形に変化させる．
	kSlope = 1 << 2,		//!< スロープ状の地形に変化させる．
	kTilt = 1 << 3,			//!< 縦軸を中心軸として回転させた地形に変化させる．
	kRough = 1 << 4,		//!< 凸凹の地形に変化させる．
};


//! @class SimulationMapCreator
//! @brief シミュレーション用のマップを生成するクラス．
//! @details MapStateクラスの初期化処理のなかで呼んで，マップを生成してくれるクラス．
//! @n 先行研究のプログラムでは mainfunction.h で実装されていた処理である．
class SimulationMapCreator final : public IMapCreator
{
public:

	//! @brief コンストラクタで作成するマップ情報を与える
	//! @n オプションはbit演算を利用して複数指定できる．例えば穴あきかつ，階段状にしたいならば，
	//! MapCreateOption::kPerforated | MapCreateOption::kStep と指定する
	//! @param [in] mode 列挙体 MapCreateMode でどのような地形を生成するか指定する．
	//! @param [in] option MapCreateOptionをstatic_cast<unsigned int>でキャストしたものを指定する．
	SimulationMapCreator(MapCreateMode mode, unsigned int option);


	MapState InitMap() override;

	void UpdateMap(MapState* current_map) override;


private:

	//! フラットなマップを生成する
	void CreateFlatMap(std::vector<designlab::Vector3>* map) const;

	//! 縦じま模様の穴が開いたマップを生成する
	void CreateVerticalStripeMap(std::vector<designlab::Vector3>* map) const;

	//! 横じま模様の穴が開いたマップを生成する
	void CreateHorizontalStripeMap(std::vector<designlab::Vector3>* map) const;

	//! 斜めじま模様の穴が開いたマップを生成する
	void CreateDiagonalStripeMap(std::vector<designlab::Vector3>* map) const;

	//! 網目模様の穴が開いたマップを生成する
	void CreateMeshMap(std::vector<designlab::Vector3>* map)  const;

	//! 網目模様の穴が開いたマップを生成する
	void CreateLatticePointMap(std::vector<designlab::Vector3>* map) const;

	//生成されたマップにランダムな穴をあける
	void ChangeMapToPerforated(std::vector<designlab::Vector3>* map) const;

	//生成されたマップを階段状にする
	void ChangeMapToStep(std::vector<designlab::Vector3>* map) const;

	//生成されたマップを坂道にする
	void ChangeMapToSlope(std::vector<designlab::Vector3>* map) const;

	//生成されたマップを傾ける
	void ChangeMapToTilt(std::vector<designlab::Vector3>* map) const;

	//生成されたマップをデコボコにする
	void ChangeMapToRough(std::vector<designlab::Vector3>* map) const;


	MapCreateMode mode_;	//!< マップ生成のモードを指定する列挙体．

	unsigned int option_;	//!< マップ生成のオプションを指定するbit．

	const float kBaseZ;		//!< マップの基準となるZ座標．

	const float kMapMaxX;	//!< マップのX座標の最大値．
	const float kMapMinX;	//!< マップのX座標の最小値．
	const float kMapMaxY;	//!< マップのY座標の最大値．
	const float kMapMinY;	//!< マップのY座標の最小値．
	const float kMapStartRoughX;	//!< 不整地が始まるX座標．
};


#endif // !DESIGNLAB_MAP_CREATOR_H_