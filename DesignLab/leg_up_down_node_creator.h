//! @file leg_up_down_node_creator.h
//! @brief 脚の上げ下げをするエッジ(辺，ノードとノードを繋ぐ物)の処理をするクラス．

#ifndef DESIGNLAB_LEG_UP_DOWN_NODE_CREATOR_H_
#define DESIGNLAB_LEG_UP_DOWN_NODE_CREATOR_H_


#include "interface_node_creator.h"

#include <memory>

#include "abstract_hexapod_state_calculator.h"
#include "hexapod_state_calculator.h"
#include "map_state.h"


//! @class LegUpDownNodeCreator
//! @brief 脚の上げ下げをするエッジ(辺，ノードとノードを繋ぐ物)の処理をするクラス．
//! @note 回転を考慮していないので注意
class LegUpDownNodeCreator final : public INodeCreator
{
public:

	LegUpDownNodeCreator(const DevideMapState& devide_map, const std::shared_ptr<const AbstractHexapodStateCalculator>& calc, const EHexapodMove next_move);
	~LegUpDownNodeCreator() = default;

	void Create(const SNode& current_node, int current_node_index, std::vector<SNode>* output_graph) override;

private:

	//脚が接地可能か調べる．地面に干渉するかどうかを調べていないので注意．実際に接地するとしたらどこになるかをoutput_ground_posで出力する．
	bool IsGroundableLeg(int leg_num, const SNode& current_node, designlab::Vector3* output_ground_pos);

	//離散化した脚位置の4のグローバル座標，候補点のグローバル座標，付け根のグローバル座標．現在の脚状態(1〜7)，これらを利用して候補点が離散化した脚位置に適しているか調べる．
	bool IsAbleLegPos(const SNode& node, int leg_num);


	const float kLegMargin;		//!< これだけ動かせば現在の脚位置でも届くのならば，脚位置4判定となる．
	const float kHighMargin;	//!< 縦方向（Z軸方向）のマージン．この範囲内ならば真ん中にあるとみなす．

	const DevideMapState map_;	//!< マップの状態．

	HexapodStateCalclator_Old m_calclator;	//ロボットの座標計算クラス．
	const std::shared_ptr<const AbstractHexapodStateCalculator> mp_calclator;	//!< ロボットの座標計算クラス．

	const EHexapodMove next_move_;	//!< 次の移動方向．
};


#endif	//DESIGNLAB_LEG_UP_DOWN_NODE_CREATOR_H_