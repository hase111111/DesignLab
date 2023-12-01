﻿#include "node_creator_leg_up_down_2d.h"

#include <algorithm>

#include <boost/dynamic_bitset.hpp>

#include "com_type.h"
#include "designlab_math_util.h"
#include "graph_search_const.h"
#include "leg_state.h"


namespace dl = ::designlab;
namespace dlcf = ::designlab::com_func;
namespace dllf = ::designlab::leg_func;
namespace dlm = ::designlab::math_util;


NodeCreatorLegUpDown2d::NodeCreatorLegUpDown2d(
	const DevideMapState& devide_map,
	const std::shared_ptr<const IHexapodCoordinateConverter>& converter_ptr,
	const std::shared_ptr<const IHexapodStatePresenter>& presenter_ptr,
	const std::shared_ptr<const IHexapodVaildChecker>& checker_ptr,
	HexapodMove next_move
) :
	kLegMargin(20),
	map_(devide_map),
	converter_ptr_(converter_ptr),
	presenter_ptr_(presenter_ptr),
	checker_ptr_(checker_ptr),
	next_move_(next_move)
{
};

void NodeCreatorLegUpDown2d::Create(const RobotStateNode& current_node, int current_node_index, std::vector<RobotStateNode>* output_graph) const
{
	assert(
		dllf::GetDiscreteLegPos(current_node.leg_state, 0) == DiscreteLegPos::kBack ||
		dllf::GetDiscreteLegPos(current_node.leg_state, 0) == DiscreteLegPos::kCenter ||
		dllf::GetDiscreteLegPos(current_node.leg_state, 0) == DiscreteLegPos::kFront
	);
	assert(
		dllf::GetDiscreteLegPos(current_node.leg_state, 1) == DiscreteLegPos::kBack ||
		dllf::GetDiscreteLegPos(current_node.leg_state, 1) == DiscreteLegPos::kCenter ||
		dllf::GetDiscreteLegPos(current_node.leg_state, 1) == DiscreteLegPos::kFront
	);
	assert(
		dllf::GetDiscreteLegPos(current_node.leg_state, 2) == DiscreteLegPos::kBack ||
		dllf::GetDiscreteLegPos(current_node.leg_state, 2) == DiscreteLegPos::kCenter ||
		dllf::GetDiscreteLegPos(current_node.leg_state, 2) == DiscreteLegPos::kFront
	);
	assert(
		dllf::GetDiscreteLegPos(current_node.leg_state, 3) == DiscreteLegPos::kBack ||
		dllf::GetDiscreteLegPos(current_node.leg_state, 3) == DiscreteLegPos::kCenter ||
		dllf::GetDiscreteLegPos(current_node.leg_state, 3) == DiscreteLegPos::kFront
	);
	assert(
		dllf::GetDiscreteLegPos(current_node.leg_state, 4) == DiscreteLegPos::kBack ||
		dllf::GetDiscreteLegPos(current_node.leg_state, 4) == DiscreteLegPos::kCenter ||
		dllf::GetDiscreteLegPos(current_node.leg_state, 4) == DiscreteLegPos::kFront
	);
	assert(
		dllf::GetDiscreteLegPos(current_node.leg_state, 5) == DiscreteLegPos::kBack ||
		dllf::GetDiscreteLegPos(current_node.leg_state, 5) == DiscreteLegPos::kCenter ||
		dllf::GetDiscreteLegPos(current_node.leg_state, 5) == DiscreteLegPos::kFront
	);

	//脚の遊脚・接地によって生じるとりうる重心をcomtypeとして仕分けている．(詳しくはcom_type.hを参照)．
	// vector<bool>を使用したいが，vector<bool>はテンプレートの特殊化で通常のvectorとは違う挙動をするので，boost::dynamic_bitset<>を使用する．
	boost::dynamic_bitset<> is_able_leg_ground_pattern(dlcf::GetLegGroundPatternNum());

	is_able_leg_ground_pattern.set();	//全てtrueにする．


	//まず離散化された重心位置から取り得ない接地パターンを除外する．
	dlcf::RemoveLegGroundPatternFromCom(dllf::GetDiscreteComPos(current_node.leg_state), &is_able_leg_ground_pattern);


	//次に脚が地面に接地可能か調べる．
	bool is_groundable_leg[HexapodConst::kLegNum];			//脚が設置可能ならばtrueになる．既に接地しているならばtrueになる．
	dl::Vector3 ground_pos[HexapodConst::kLegNum];	//脚が接地する座標．

	for (int i = 0; i < HexapodConst::kLegNum; i++) { ground_pos[i] = current_node.leg_pos[i]; }

	for (int i = 0; i < HexapodConst::kLegNum; i++)
	{
		if (dllf::IsGrounded(current_node.leg_state, i))
		{
			//すでに接地している脚は接地可能に決まっているのでtrueにする．
			is_groundable_leg[i] = true;
			ground_pos[i] = current_node.leg_pos[i];
		}
		else
		{
			//現在遊脚中の脚は自身の脚状態で接地できるか検討する．
			dl::Vector3 res_ground_pos;

			if (IsGroundableLeg(i, current_node, &res_ground_pos))
			{
				is_groundable_leg[i] = true;	//接地可能にする．
				ground_pos[i] = res_ground_pos;
			}
			else
			{
				is_groundable_leg[i] = false;	//接地不可能にする．
				dlcf::RemoveLegGroundPatternFromNotGroundableLeg(i, &is_able_leg_ground_pattern);
			}
		}
	}


	//子ノードを生成する．
	for (int i = 0; i < dlcf::GetLegGroundPatternNum(); i++)
	{
		//その重心タイプが可能であれば，追加する
		if (is_able_leg_ground_pattern[i])
		{
			RobotStateNode res_node = current_node;

			res_node.ChangeToNextNode(current_node_index, next_move_);


			//遊脚・接地を書き換える．
			dllf::LegGroundedBit new_is_ground = dlcf::GetLegGroundedBitFromLegGroundPatternIndex(i);

			dllf::ChangeAllLegGround(new_is_ground, &res_node.leg_state);


			//脚位置を書き換える．
			for (int j = 0; j < HexapodConst::kLegNum; j++)
			{
				if (new_is_ground[j])
				{
					res_node.leg_pos[j] = ground_pos[j];

					res_node.leg_reference_pos[j] = ground_pos[j];
				}
				else
				{
					res_node.leg_pos[j] = presenter_ptr_->GetFreeLegPosLegCoodinate(j);

					res_node.leg_reference_pos[j].x = res_node.leg_pos[j].x;
					res_node.leg_reference_pos[j].y = res_node.leg_pos[j].y;
				}
			}

			if (checker_ptr_->IsStable(res_node.leg_state, res_node.leg_pos))
			{
				(*output_graph).push_back(res_node);
			}

		}	//if is_able_leg_ground_pattern[i]

	}	//for i
}

bool NodeCreatorLegUpDown2d::IsGroundableLeg(int now_leg_num, const RobotStateNode& current_node, designlab::Vector3* output_ground_pos) const
{
	//for文の中のcontinueについては http://www9.plala.or.jp/sgwr-t/c/sec06-7.html を参照．

	//脚座標がdevide mapでどこに当たるか調べて，そのマスの2つ上と2つ下の範囲内を全て探索する．
	const dl::Vector3 kGlobalLegbasePos = converter_ptr_->ConvertLegToGlobalCoordinate(
		current_node.leg_reference_pos[now_leg_num],
		now_leg_num,
		current_node.global_center_of_mass,
		current_node.quat,
		true
	);


	int max_x_dev = map_.GetDevideMapIndexX(kGlobalLegbasePos.x) + 2;
	int min_x_dev = map_.GetDevideMapIndexX(kGlobalLegbasePos.x) - 2;
	int max_y_dev = map_.GetDevideMapIndexY(kGlobalLegbasePos.y) + 2;
	int min_y_dev = map_.GetDevideMapIndexY(kGlobalLegbasePos.y) - 2;

	//値がdevide mapの範囲外にあるときは丸める．
	max_x_dev = DevideMapState::ClampDevideMapIndex(max_x_dev);
	min_x_dev = DevideMapState::ClampDevideMapIndex(min_x_dev);
	max_y_dev = DevideMapState::ClampDevideMapIndex(max_y_dev);
	min_y_dev = DevideMapState::ClampDevideMapIndex(min_y_dev);

	//devide map内を全探索して，現在の脚位置(離散化した物)に適した脚設置可能点が存在するか調べる．
	dl::Vector3 candidate_pos;	//現在の脚位置に合致する候補座標群．
	bool is_candidate_pos = false;		//候補座標が存在するかどうか．

	//範囲内の点を全て調べる．
	for (int x = min_x_dev; x < max_x_dev; x++)
	{
		for (int y = min_y_dev; y < max_y_dev; y++)
		{
			const int kPosNum = map_.GetPointNum(x, y);

			for (int n = 0; n < kPosNum; n++)
			{
				dl::Vector3 map_point_pos = map_.GetPointPos(x, y, n);	//脚設置可能点の座標を取り出す．
				map_point_pos = converter_ptr_->ConvertGlobalToLegCoordinate(
					map_point_pos,
					now_leg_num,
					current_node.global_center_of_mass,
					current_node.quat,
					true
				);

				//脚位置を更新したノードを作成する．
				RobotStateNode new_node = current_node;

				new_node.leg_pos[now_leg_num] = map_point_pos;


				//前の候補地点と比較して，より良い候補地点の時のみ実行すする
				if (is_candidate_pos)
				{
					//反対方向をむいている場合は候補地点として採用しない．
					if (new_node.leg_reference_pos[now_leg_num].ProjectedXY().Cross(candidate_pos.ProjectedXY()) * new_node.leg_reference_pos[now_leg_num].ProjectedXY().Cross(map_point_pos.ProjectedXY()) < 0)
					{
						continue;
					}

					//現在の脚位置と候補地点の間に障害物がある場合は候補地点として採用しない．
					if (map_point_pos.ProjectedXY().Cross(candidate_pos.ProjectedXY()) * map_point_pos.ProjectedXY().Cross(new_node.leg_reference_pos[now_leg_num].ProjectedXY()) < 0)
					{
						continue;
					}
				}

				dllf::ChangeGround(now_leg_num, true, &new_node.leg_state);

				if (!checker_ptr_->IsLegInRange(now_leg_num, new_node.leg_pos[now_leg_num])) { continue; }	//脚が範囲外ならば追加せずに続行．

				if (!IsAbleLegPos(new_node, now_leg_num)) { continue; }	//候補座標として，適していないならば追加せずに続行．

				is_candidate_pos = true;
				candidate_pos = map_point_pos;
			}

		}	//for y

	}	//for x


	//候補点を全列挙したのち，候補点が一つもなければfalse
	if (!is_candidate_pos) { return false; }

	//存在するなら，その中で最も適したものを結果として返し，true
	(*output_ground_pos) = candidate_pos;

	return true;
}

bool NodeCreatorLegUpDown2d::IsAbleLegPos(const RobotStateNode& node, int leg_index) const
{
	const DiscreteLegPos discrete_leg_pos = dllf::GetDiscreteLegPos(node.leg_state, leg_index);		//脚位置を取得

	//まず最初に脚位置4のところにないか確かめる．
	if ((node.leg_reference_pos[leg_index] - node.leg_pos[leg_index]).GetSquaredLength() < dlm::Squared(kLegMargin))
	{
		if (discrete_leg_pos == DiscreteLegPos::kCenter) { return true; }
		else { return false; }
	}
	else
	{
		if (discrete_leg_pos == DiscreteLegPos::kCenter) { return false; }
	}

	//脚位置4と比較して前か後ろか
	if (node.leg_reference_pos[leg_index].ProjectedXY().Cross(node.leg_pos[leg_index].ProjectedXY()) * node.leg_pos[leg_index].ProjectedXY().Cross({ 1,0 }) > 0)
	{
		//前
		if (discrete_leg_pos == DiscreteLegPos::kBack)
		{
			return false;
		}
	}
	else
	{
		//後ろ
		if (discrete_leg_pos == DiscreteLegPos::kFront)
		{
			return false;
		}
	}

	return true;
}