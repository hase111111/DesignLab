#include "com_move_node_creator_hato.h"

#include "designlab_math_util.h"
#include "graph_search_const.h"
#include "leg_state.h"


namespace dllf = designlab::leg_func;
namespace dlm = designlab::math_util;


ComMoveNodeCreatorHato::ComMoveNodeCreatorHato(const DevideMapState& map, const std::shared_ptr<const AbstractHexapodStateCalculator>& calc, const HexapodMove next_move) :
	kStableMargin(15.0f),
	map_(map), 
	calculator_ptr_(calc), 
	maker_(calc),
	selecter_(calc),
	next_move_(next_move)
{
}


void ComMoveNodeCreatorHato::Create(const RobotStateNode& current_node, const int current_num, std::vector<RobotStateNode>* output_graph)
{
	std::array<ComPosAndPolygon, ComCandidatePolygonMaker::MAKE_POLYGON_NUM> candidate_polygons;

	//�d�S�ړ���̌��n�_�͈̔͂��������p�`���쐬����
	maker_.MakeCandidatePolygon(current_node, &candidate_polygons);

	//���͈͂�����ۂɈړ������̍��W��I������
	selecter_.SetCurrentNode(current_node);

	for (int i = 0; i < ComCandidatePolygonMaker::MAKE_POLYGON_NUM; ++i)
	{
		//�����������p�`�����_�ɂȂ肦�Ȃ��Ȃ�΁C���̑��p�`�͖�������
		if (!candidate_polygons[i].is_able) { continue; }

		designlab::Vector3 result_com;

		if (selecter_.GetComFromPolygon(candidate_polygons[i].polygon, &result_com))
		{
			RobotStateNode next_node = current_node;

			next_node.ChangeGlobalCenterOfMass(result_com, false);					//�d�S�ʒu��ύX���C����ɔ����ڒn�r�̈ʒu���ύX����

			dllf::ChangeDiscreteComPos(candidate_polygons[i].com_pos, &next_node.leg_state);		//leg_state��com_pattern��ύX����

			for (int j = 0; j < HexapodConst::kLegNum; ++j)
			{
				dllf::ChangeDiscreteLegPos(j, DiscreteLegPos::kCenter, &next_node.leg_state);
			}

			next_node.ChangeToNextNode(current_num, next_move_);	//�[����e�m�[�h��ύX����

			if (IsStable(next_node) && !IsIntersectGround(next_node))
			{
				(*output_graph).push_back(next_node);
			}
		}
	}
}


bool ComMoveNodeCreatorHato::IsStable(const RobotStateNode& node) const
{
	//�d�S�����_�Ƃ������W�n�ŁC�r�̈ʒu���v�Z����D

	if (calculator_ptr_->CalculateStabilityMargin(node.leg_state, node.leg_pos) < kStableMargin)
	{
		return false;
	}
	else
	{
		return true;
	}
}


bool ComMoveNodeCreatorHato::IsIntersectGround(const RobotStateNode& node) const
{
	float top_z = -10000.0f;	//�n�ʂƂ̌�_�̂����ł��������̂��i�[����

	for (int i = 0; i < HexapodConst::kLegNum; i++)
	{
		const designlab::Vector3 kCoxaPos = calculator_ptr_->GetGlobalLegBasePosition(i, node.global_center_of_mass, node.rot, false);	//�r�̍����̍��W(�O���[�o��)���擾����

		if (map_.IsInMap(kCoxaPos)) 
		{
			const float kMapTopZ = map_.GetTopZ(map_.GetDevideMapIndexX(kCoxaPos.x), map_.GetDevideMapIndexY(kCoxaPos.y));

			top_z = (std::max)(top_z, kMapTopZ);	//�ł������_�����߂�		
		}
	}

	if (top_z + HexapodConst::VERTICAL_MIN_RANGE - dlm::kAllowableError < node.global_center_of_mass.z)
	{
		return false;
	}

	return true;
}
