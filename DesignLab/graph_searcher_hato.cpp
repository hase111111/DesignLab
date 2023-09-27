#include "graph_searcher_hato.h"

#include <iostream>
#include <cmath>

#include "designlab_math_util.h"
#include "graph_search_const.h"
#include "leg_state.h"


namespace dllf = designlab::leg_func;
namespace dlm = designlab::math_util;


GraphSearcherHato::GraphSearcherHato(const std::shared_ptr<const AbstractHexapodStateCalculator>& calc) :
	mp_calculator(calc)
{
	if (GraphSearchConst::DO_DEBUG_PRINT)
	{
		std::cout << "[GraphSearcher] GraphSearcherHato : コンストラクタが呼ばれた" << std::endl;
	}
}

GraphSearcherHato::~GraphSearcherHato()
{
	if (GraphSearchConst::DO_DEBUG_PRINT)
	{
		std::cout << "[GraphSearcher] GraphSearcherHato : デストラクタが呼ばれた" << std::endl;
	}
}

GraphSearchResult GraphSearcherHato::SearchGraphTree(const std::vector<SNode>& graph, const STarget& target, SNode* output_result)
{
	if (GraphSearchConst::DO_DEBUG_PRINT)
	{
		std::cout << "[GraphSearcher] GraphSearcherHato : searchGraphTree() 探索開始\n";
	}

	// _targetの値によって，探索方法を変える必要がある．探索方法を抽象化するべき．

	// @todo initializerで初期化する処理を書く

	// ターゲットモードが直進と仮定して処理を書いている

	int result_index = -1;
	float max_rot_angle = 0;
	float max_leg_rot_angle = 0;
	float max_margin = 0;
	float min_leg_dif = 0;

	const size_t kGraphSize = graph.size();
	size_t parent_num = getParentNodeIndex(graph);

	if (parent_num < 0) { return GraphSearchResult::FailureByNoNode; }

	initEvaluationValue(graph.at(parent_num), target);

	for (size_t i = 0; i < kGraphSize; i++)
	{
		//最大深さのノードのみを評価する
		if (graph[i].depth == GraphSearchConst::MAX_DEPTH)
		{
			//結果が見つかっていない場合は，必ず結果を更新する
			if (result_index < 0)
			{
				result_index = static_cast<int>(i);
				max_rot_angle = calcMoveFrowardEvaluationValue(graph[i], target);
				max_leg_rot_angle = calcLegRotEvaluationValue(graph[i], target);
				max_margin = mp_calculator->CalculateStabilityMargin(graph[i].leg_state, graph[i].leg_pos);
				min_leg_dif = abs(graph[i].global_center_of_mass.z - graph[parent_num].global_center_of_mass.z);
				continue;
			}

			float candiate_rot_angle = calcMoveFrowardEvaluationValue(graph[i], target);
			float candiate_leg_rot_angle = calcLegRotEvaluationValue(graph[i], target);
			float candiate_margin = mp_calculator->CalculateStabilityMargin(graph[i].leg_state, graph[i].leg_pos);
			float candiate_leg_dif = abs(graph[i].global_center_of_mass.z - graph[parent_num].global_center_of_mass.z);

			if (max_rot_angle < candiate_rot_angle)
			{
				max_rot_angle = candiate_rot_angle;
				max_leg_rot_angle = candiate_leg_rot_angle;
				max_margin = candiate_margin;
				min_leg_dif = candiate_leg_dif;
				result_index = static_cast<int>(i);
			}
			else if (dlm::IsEqual(max_rot_angle, candiate_rot_angle))
			{
				if (min_leg_dif > candiate_leg_dif)
				{
					max_rot_angle = candiate_rot_angle;
					max_leg_rot_angle = candiate_leg_rot_angle;
					max_margin = candiate_margin;
					min_leg_dif = candiate_leg_dif;
					result_index = static_cast<int>(i);
				}
				else if (dlm::IsEqual(min_leg_dif, candiate_leg_dif))
				{
					if (max_leg_rot_angle < candiate_leg_rot_angle)
					{
						max_rot_angle = candiate_rot_angle;
						max_leg_rot_angle = candiate_leg_rot_angle;
						max_margin = candiate_margin;
						min_leg_dif = candiate_leg_dif;
						result_index = static_cast<int>(i);
					}
					else if (dlm::IsEqual(max_leg_rot_angle, candiate_leg_rot_angle))
					{
						if (max_margin < candiate_margin)
						{
							max_rot_angle = candiate_rot_angle;
							max_leg_rot_angle = candiate_leg_rot_angle;
							max_margin = candiate_margin;
							min_leg_dif = candiate_leg_dif;
							result_index = static_cast<int>(i);
						}
					}
				}

			}
		}
	}

	// index が範囲外ならば失敗
	if (result_index < 0 || result_index >= kGraphSize) { return GraphSearchResult::FailureByNoNode; }

	//深さ1まで遡って値を返す
	if (getDepth1NodeFromMaxDepthNode(graph, result_index, output_result) == false) { return GraphSearchResult::FailureByNotReachedDepth; }

	if (GraphSearchConst::DO_DEBUG_PRINT)
	{
		std::cout << "[GraphSearcher] GraphSearcherHato : searchGraphTree() 探索終了" << std::endl;
	}

	return GraphSearchResult::Success;
}

size_t GraphSearcherHato::getParentNodeIndex(const std::vector<SNode>& graph) const
{
	const size_t kGraphSize = graph.size();
	size_t parent_num = 0;

	for (size_t i = 0; i < kGraphSize; i++)
	{
		if (graph.at(i).depth == 0)
		{
			parent_num = i;
			break;
		}
	}

	return parent_num;
}

bool GraphSearcherHato::getDepth1NodeFromMaxDepthNode(const std::vector<SNode>& graph, const size_t max_depth_node_index, SNode* output_node) const
{
	size_t result_index = max_depth_node_index;
	const size_t kGraphSize = graph.size();
	int count = 0;

	while (graph.at(result_index).depth != 1)
	{
		result_index = graph.at(result_index).parent_num;

		if (result_index < 0 || result_index >= kGraphSize) { return false; }

		count++;
		if (count > GraphSearchConst::MAX_DEPTH) { return false; }
	}

	(*output_node) = graph.at(result_index);
	return true;
}

void GraphSearcherHato::initEvaluationValue(const SNode& parent_node, const STarget& target)
{
	m_parent_node = parent_node;

	//警告回避用
	STarget target_copy = target;
}

float GraphSearcherHato::calcMoveFrowardEvaluationValue(const SNode& current_node, const STarget& target) const
{
	// 警告回避用
	STarget target_copy = target;

	//designlab::Vector3 center_com_dif = current_node.global_center_of_mass - target.TargetPosition;
	//designlab::Vector3 m_target_to_parent = m_parent_node.global_center_of_mass - target.TargetPosition;

	//return (int)(m_target_to_parent.ProjectedXY().Length() - center_com_dif.ProjectedXY().Length()) / 10 * 10.0f;

	designlab::Vector3 target_pos {10000, 0, 0};
	designlab::Vector3 target_to_parent = current_node.global_center_of_mass - target_pos;

	return target_pos.Length() - target_to_parent.Length();
}

float GraphSearcherHato::calcLegRotEvaluationValue(const SNode& current_node, const STarget& target) const
{
	// 警告回避用
	STarget target_copy = target;

	float result = 0.0f;

	for (int i = 0; i < HexapodConst::LEG_NUM; i++)
	{
		if (dllf::IsGrounded(current_node.leg_state, i))
		{
			result += (current_node.leg_pos[i] - m_parent_node.leg_pos[i]).Length();
		}
	}

	return result / (float)HexapodConst::LEG_NUM;
}
