#include "com_move_node_creator_hato.h"

#include "designlab_math_util.h"
#include "graph_search_const.h"
#include "leg_state.h"

namespace dlm = designlab::math_util;


ComMoveNodeCreatorHato::ComMoveNodeCreatorHato(const DevideMapState& map, const std::shared_ptr<const AbstractHexapodStateCalculator>& calc, const EHexapodMove next_move) :
	kStableMargin(15.0f),
	map_(map), 
	calculator_ptr_(calc), 
	maker_(calc),
	selecter_(calc),
	next_move_(next_move)
{
}


void ComMoveNodeCreatorHato::Create(const SNode& current_node, const int current_num, std::vector<SNode>* output_graph)
{
	std::pair<designlab::Polygon2, EDiscreteComPos> candidate_polygons[ComCandidatePolygonMaker::MAKE_POLYGON_NUM];

	//�d�S�ړ���̌��n�_�͈̔͂��������p�`���쐬����
	maker_.makeCandidatePolygon(current_node, candidate_polygons);

	//���͈͂�����ۂɈړ������̍��W��I������
	selecter_.setCurrentNode(current_node);

	for (int i = 0; i < ComCandidatePolygonMaker::MAKE_POLYGON_NUM; ++i)
	{
		//�����������p�`�����_�ɂȂ肦�Ȃ��Ȃ�΁C���̑��p�`�͖�������
		if (candidate_polygons[i].second == EDiscreteComPos::ERROR_POS) { continue; }

		designlab::Vector3 result_com;

		if (selecter_.getComFromPolygon(candidate_polygons[i].first, /*candidate_polygons[i].second,*/ &result_com))
		{
			SNode next_node = current_node;

			next_node.changeGlobalCenterOfMass(result_com, false);					//�d�S�ʒu��ύX���C����ɔ����ڒn�r�̈ʒu���ύX����

			dl_leg::changeComPattern(candidate_polygons[i].second, &next_node.leg_state);		//leg_state��com_pattern��ύX����

			for (int j = 0; j < HexapodConst::LEG_NUM; ++j)
			{
				dl_leg::changeLegStateKeepTopBit(j, EDiscreteLegPos::CENTER, &next_node.leg_state);
			}

			next_node.changeNextNode(current_num, next_move_);	//�[����e�m�[�h��ύX����

			if (isStable(next_node) && !isIntersectGround(next_node))
			{
				(*output_graph).push_back(next_node);
			}
		}
	}
}


bool ComMoveNodeCreatorHato::isStable(const SNode& node) const
{
	//�d�S�����_�Ƃ������W�n�ŁC�r�̈ʒu���v�Z����D

	if (calculator_ptr_->calcStabilityMargin(node.leg_state, node.leg_pos) < kStableMargin)
	{
		return false;
	}
	else
	{
		return true;
	}
}


bool ComMoveNodeCreatorHato::isIntersectGround(const SNode& node) const
{
	float top_z = -10000.0f;	//�n�ʂƂ̌�_�̂����ł��������̂��i�[����

	for (int i = 0; i < HexapodConst::LEG_NUM; i++)
	{
		const designlab::Vector3 kCoxaPos = calculator_ptr_->getGlobalLegBasePosition(i, node.global_center_of_mass, node.rot, false);	//�r�̍����̍��W(�O���[�o��)���擾����

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
