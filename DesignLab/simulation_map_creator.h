//! @file map_creator.h
//! @brief マップ生成クラス．


#ifndef DESIGNLAB_MAP_CREATOR_H_
#define DESIGNLAB_MAP_CREATOR_H_


#include <vector>

#include "designlab_vector3.h"
#include "interface_map_creator.h"
#include "map_state.h"
#include "designlab_attribute.h"
#include "map_create_mode_messenger.h"


//! @class SimulationMapCreator
//! @brief シミュレーション用のマップを生成するクラス．
//! @details MapStateクラスの初期化処理のなかで呼んで，マップを生成してくれるクラス．
//! @n 先行研究のプログラムでは mainfunction.h で実装されていた処理である．
class SimulationMapCreator final : public IMapCreator
{
public:

	//! @brief コンストラクタで作成するマップ情報を与える
	//! @param [in] messanger マップ生成のモードとオプションを指定する構造体．
	SimulationMapCreator(const MapCreateModeMessenger& messanger);


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

	//! 生成されたマップにランダムな穴をあける
	void ChangeMapToPerforated(std::vector<designlab::Vector3>* map) const;

	//! 生成されたマップを階段状にする
	void ChangeMapToStep(std::vector<designlab::Vector3>* map) const;

	//! 生成されたマップを坂道にする
	void ChangeMapToSlope(std::vector<designlab::Vector3>* map) const;

	//! 生成されたマップを傾ける
	void ChangeMapToTilt(std::vector<designlab::Vector3>* map) const;

	//! 生成されたマップをデコボコにする
	void ChangeMapToRough(std::vector<designlab::Vector3>* map) const;


	const MapCreateModeMessenger messanger_;	//!< マップ生成のモードを指定する列挙体．
};


#endif // DESIGNLAB_MAP_CREATOR_H_