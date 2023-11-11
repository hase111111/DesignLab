#include "com_move_node_creator_hato.h"

#include "designlab_math_util.h"
#include "graph_search_const.h"
#include "leg_state.h"


namespace dllf = designlab::leg_func;
namespace dlm = designlab::math_util;


ComMoveNodeCreatorHato::ComMoveNodeCreatorHato(
	const DevideMapState& map, 
	const std::shared_ptr<const IHexapodCoordinateConverter>& converter_ptr,
	const std::shared_ptr<const IHexapodStatePresenter>& presenter_ptr,
	const std::shared_ptr<const IHexapodVaildChecker>& checker_ptr,
	const HexapodMove next_move
) :
	kStableMargin(15.0f),
	map_(map), 
	maker_(converter_ptr),
	selecter_(checker_ptr),
	next_move_(next_move),
	converter_ptr_(converter_ptr),
	presenter_ptr_(presenter_ptr),
	checker_ptr_(checker_ptr)
{
}


void ComMoveNodeCreatorHato::Create(const RobotStateNode& current_node, const int current_num, std::vector<RobotStateNode>* output_graph) const
{
	std::array<ComPosAndPolygon, ComCandidatePolygonMaker::MAKE_POLYGON_NUM> candidate_polygons;

	//�d�S�ړ���̌��n�_�͈̔͂��������p�`���쐬����
	maker_.MakeCandidatePolygon(current_node, &candidate_polygons);

	//���͈͂�����ۂɈړ������̍��W��I������
	for (int i = 0; i < ComCandidatePolygonMaker::MAKE_POLYGON_NUM; ++i)
	{
		//�����������p�`�����_�ɂȂ肦�Ȃ��Ȃ�΁C���̑��p�`�͖�������
		if (!candidate_polygons[i].is_able) { continue; }

		designlab::Vector3 result_com;

		if (selecter_.GetComFromPolygon(candidate_polygons[i].polygon, current_node, &result_com))
		{
			RobotStateNode next_node = current_node;

			next_node.ChangeGlobalCenterOfMass(result_com, false);					//�d�S�ʒu��ύX���C����ɔ����ڒn�r�̈ʒu���ύX����

			dllf::ChangeDiscreteComPos(candidate_polygons[i].com_pos, &next_node.leg_state);		//leg_state��com_pattern��ύX����

			for (int j = 0; j < HexapodConst::kLegNum; ++j)
			{
				dllf::ChangeDiscreteLegPos(j, DiscreteLegPos::kCenter, &next_node.leg_state);
			}

			next_node.ChangeToNextNode(current_num, next_move_);	//�[����e�m�[�h��ύX����

			if (
				checker_ptr_->IsStable(next_node.leg_state, next_node.leg_pos) &&
				! checker_ptr_->IsBodyInterferingWithGround(next_node, map_)
			)
			{
				(*output_graph).push_back(next_node);
			}
		}
	}
}