﻿#include "graph_searcher_straight_move.h"

#include <functional>
#include <vector>

#include "cassert_define.h"
#include "math_util.h"
#include "graph_search_const.h"
#include "leg_state.h"


namespace designlab
{

GraphSearcherStraightMove::GraphSearcherStraightMove(const std::shared_ptr<const IHexapodVaildChecker>& checker_ptr) :
	checker_ptr_(checker_ptr)
{
}

std::tuple<GraphSearchResult, int, int> GraphSearcherStraightMove::SearchGraphTree(
	const GaitPatternGraphTree& graph,
	const RobotOperation& operation,
	[[maybe_unused]] const DevideMapState& devide_map_state,
	const int max_depth
) const
{
	assert(operation.operation_type == enums::RobotOperationType::kStraightMovePosition ||
		   operation.operation_type == enums::RobotOperationType::kStraightMoveVector);	//ターゲットモードは直進である．

	if (!graph.HasRoot())
	{
		const GraphSearchResult result = { enums::Result::kFailure, "ルートノードがありません．" };
		return { result, -1, -1 };
	}

	//初期化
	EvaluationValue max_evaluation_value;
	InitialValue init_value;

	if (operation.operation_type == enums::RobotOperationType::kStraightMovePosition)
	{
		init_value.normalized_move_direction = (operation.straight_move_position_ - graph.GetRootNode().global_center_of_mass);
		init_value.normalized_move_direction.z = 0.0f;
		init_value.normalized_move_direction = init_value.normalized_move_direction.GetNormalized();
	}
	else
	{
		init_value.normalized_move_direction = operation.straight_move_vector_;
		init_value.normalized_move_direction.z = 0.0f;
		init_value.normalized_move_direction = init_value.normalized_move_direction.GetNormalized();
	}

	init_value.target_z_value = InitTargetZValue(graph.GetRootNode(), devide_map_state, init_value.normalized_move_direction);

	std::cout << "init_value.target_z_value:" << init_value.target_z_value << std::endl;


	//Calcなどの関数をvectorに格納する．
	std::vector<std::function<EvaluationResult(const int, const GaitPatternGraphTree&, const EvaluationValue&, const InitialValue&, EvaluationValue*)>> update_evaluation_value_func_vec
	{
		std::bind(&GraphSearcherStraightMove::UpdateEvalutionValueByZDiff, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5),
		std::bind(&GraphSearcherStraightMove::UpdateEvaluationValueByAmoutOfMovement, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5),
		std::bind(&GraphSearcherStraightMove::UpdateEvalutionValueByLegRot, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5),
		std::bind(&GraphSearcherStraightMove::UpdateEvalutionValueByStablyMargin, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5)
	};

	for (int i = 0; i < graph.GetGraphSize(); i++)
	{
		//最大深さのノードのみを評価する
		if (graph.GetNode(i).depth == max_depth)
		{
			EvaluationValue candiate_evaluation_value;
			candiate_evaluation_value.index = i;

			bool do_update = false;

			for (size_t j = 0; j < update_evaluation_value_func_vec.size(); ++j)
			{
				const EvaluationResult result = update_evaluation_value_func_vec[j](i, graph, max_evaluation_value, init_value, &candiate_evaluation_value);

				if (max_evaluation_value.index != -1)
				{
					if (result == EvaluationResult::kUpdate)
					{
						do_update = true;
					}

					if (!do_update && result == EvaluationResult::kNotUpdate)
					{
						break;
					}
				}
				else
				{
					do_update = true;
				}
			}

			if (do_update)
			{
				max_evaluation_value = candiate_evaluation_value;
			}
		}
	}

	// index が範囲外ならば失敗
	if (max_evaluation_value.index < 0 || max_evaluation_value.index >= graph.GetGraphSize())
	{
		std::string error_message = "最大評価値のインデックスが範囲外です．" + std::to_string(max_evaluation_value.index);
		const GraphSearchResult result = { enums::Result::kFailure,error_message };
		return { result, -1, -1 };
	}

	const GraphSearchResult result = { enums::Result::kSuccess, max_evaluation_value.ToString() };

	return { result, graph.GetParentNodeIndex(max_evaluation_value.index, 1), max_evaluation_value.index };
}

std::string GraphSearcherStraightMove::EvaluationValue::ToString() const
{
	std::string result = "index:" + std::to_string(index) + "/";
	result += "move_forward:" + math_util::ConvertFloatToString(move_forward, 3, 7) + "/";
	result += "leg_rot:" + math_util::ConvertFloatToString(leg_rot, 3, 7) + "/";
	result += "z_diff:" + math_util::ConvertFloatToString(z_diff, 3, 7) + "/";
	result += "stably_margin:" + math_util::ConvertFloatToString(stably_margin, 3, 7);

	return result;
}

float GraphSearcherStraightMove::InitTargetZValue(const RobotStateNode& node, const DevideMapState& devide_map_state, const Vector3& move_direction) const
{
	const float move_length = 100.0f;

	const Vector3 target_position = move_direction * move_length;

	const int div = 50;
	const float min_z = -150.0f;
	const float max_z = 150.0f;

	for (int i = 0; i < div; i++)
	{
		const float z = min_z + (max_z - min_z) / static_cast<float>(div) * static_cast<float>(i);

		Vector3 pos = node.global_center_of_mass;
		pos += target_position;
		pos.z += z;

		RobotStateNode temp_node = node;
		temp_node.ChangeGlobalCenterOfMass(pos, false);

		if (!checker_ptr_->IsBodyInterferingWithGround(temp_node, devide_map_state))
		{
			return node.global_center_of_mass.z + z;
		}
	}

	return node.global_center_of_mass.z;
}

GraphSearcherStraightMove::EvaluationResult GraphSearcherStraightMove::UpdateEvaluationValueByAmoutOfMovement(
	const int index,
	const GaitPatternGraphTree& tree,
	const EvaluationValue& max_evaluation_value,
	const InitialValue& init_value,
	EvaluationValue* candiate
) const
{
	assert(math_util::IsEqual(init_value.normalized_move_direction.GetSquaredLength(), 1.f));	//正規化されていることを確認する．
	assert(0 <= index && index < tree.GetGraphSize());	//indexが範囲内であることを確認する．
	assert(candiate != nullptr);	//candiateがnullptrでないことを確認する．

	const Vector3 root_to_current = tree.GetNode(index).global_center_of_mass - tree.GetRootNode().global_center_of_mass;

	//root_to_currentのinit_value.normalized_move_direction方向の成分を取り出す．
	const float result = root_to_current.Dot(init_value.normalized_move_direction);
	const float margin = 7.5f;

	if (max_evaluation_value.move_forward + margin < result)
	{
		(*candiate).move_forward = result;
		return EvaluationResult::kUpdate;
	}
	else if (abs(max_evaluation_value.move_forward - result) < margin)
	{
		(*candiate).move_forward = result;
		return EvaluationResult::kEqual;
	}

	return EvaluationResult::kNotUpdate;
}

GraphSearcherStraightMove::EvaluationResult GraphSearcherStraightMove::UpdateEvalutionValueByLegRot(
	const int index,
	const GaitPatternGraphTree& tree,
	const EvaluationValue& max_evaluation_value,
	[[maybe_unused]] const InitialValue& init_value,
	EvaluationValue* candiate
) const
{
	assert(0 <= index && index < tree.GetGraphSize());	//indexが範囲内であることを確認する．
	assert(candiate != nullptr);	//candiateがnullptrでないことを確認する．

	float result = 0.0f;

	for (int i = 0; i < HexapodConst::kLegNum; i++)
	{
		if (leg_func::IsGrounded(tree.GetNode(index).leg_state, i))
		{
			result += (tree.GetNode(index).leg_pos[i].ProjectedXY() - tree.GetRootNode().leg_pos[i].ProjectedXY()).GetLength();
		}
		else
		{
			result += (tree.GetNode(index).leg_pos[i] - tree.GetRootNode().leg_pos[i]).GetLength();
		}
	}

	const float margin = 10.0f;

	if (max_evaluation_value.leg_rot + margin < result)
	{
		(*candiate).leg_rot = result;
		return EvaluationResult::kUpdate;
	}
	else if (abs(max_evaluation_value.leg_rot - result) < margin)
	{
		(*candiate).leg_rot = result;
		return EvaluationResult::kEqual;
	}

	return EvaluationResult::kNotUpdate;
}

GraphSearcherStraightMove::EvaluationResult GraphSearcherStraightMove::UpdateEvalutionValueByStablyMargin(
	const int index,
	const GaitPatternGraphTree& tree,
	const EvaluationValue& max_evaluation_value,
	[[maybe_unused]] const InitialValue& init_value,
	EvaluationValue* candiate
) const
{
	assert(0 <= index && index < tree.GetGraphSize());	//indexが範囲内であることを確認する．
	assert(candiate != nullptr);	//candiateがnullptrでないことを確認する．

	const float result = checker_ptr_->CalculateStabilityMargin(tree.GetNode(index).leg_state, tree.GetNode(index).leg_pos);

	if (max_evaluation_value.stably_margin < result)
	{
		(*candiate).stably_margin = result;
		return EvaluationResult::kUpdate;
	}
	else if (max_evaluation_value.stably_margin == result)
	{
		(*candiate).stably_margin = result;
		return EvaluationResult::kEqual;
	}

	return EvaluationResult::kNotUpdate;
}

GraphSearcherStraightMove::EvaluationResult GraphSearcherStraightMove::UpdateEvalutionValueByZDiff(
	const int index,
	const GaitPatternGraphTree& tree,
	const EvaluationValue& max_evaluation_value,
	[[maybe_unused]] const InitialValue& init_value,
	EvaluationValue* candiate
) const
{
	assert(0 <= index && index < tree.GetGraphSize());	//indexが範囲内であることを確認する．
	assert(candiate != nullptr);	//candiateがnullptrでないことを確認する．

	//const float result = abs(tree.GetNode(index).global_center_of_mass.z - tree.GetRootNode().global_center_of_mass.z);
	const float result = abs(tree.GetNode(index).global_center_of_mass.z - init_value.target_z_value);

	if (result < max_evaluation_value.z_diff)
	{
		(*candiate).z_diff = result;
		return EvaluationResult::kUpdate;
	}
	else if (result == max_evaluation_value.z_diff)
	{
		(*candiate).z_diff = result;
		return EvaluationResult::kEqual;
	}

	return EvaluationResult::kNotUpdate;
}

}  // namespace designlab