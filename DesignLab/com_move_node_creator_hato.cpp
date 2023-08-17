#include "com_move_node_creator_hato.h"

#include <iostream>

#include "graph_search_const.h"
#include "leg_state.h"


ComMoveNodeCreatorHato::ComMoveNodeCreatorHato(const MapState* const p_map, const EHexapodMove next_move) : INodeCreator(p_map, next_move), mp_map(p_map)
{
	if (GraphSearchConst::DO_DEBUG_PRINT)
	{
		std::cout << "[NodeCreator] ComMoveNodeCreatorHato : �R���X�g���N�^���Ă΂ꂽ\n";
	}
}


ComMoveNodeCreatorHato::~ComMoveNodeCreatorHato()
{
	if (GraphSearchConst::DO_DEBUG_PRINT)
	{
		std::cout << "[NodeCreator] ComMoveNodeCreatorHato : �f�X�g���N�^���Ă΂ꂽ\n";
	}
}


void ComMoveNodeCreatorHato::create(const SNode& current_node, const int current_num, std::vector<SNode>* output_graph)
{
	std::pair<my_vec::SPolygon2, ComType::EComPattern> candidate_polygons[ComCandidatePolygonMaker::MAKE_POLYGON_NUM];

	//�d�S�ړ���̌��n�_�͈̔͂��������p�`���쐬����
	m_maker.makeCandidatePolygon(current_node, candidate_polygons);

	//���͈͂�����ۂɈړ������̍��W��I������
	m_selecter.setCurrentNode(current_node);

	for (int i = 0; i < ComCandidatePolygonMaker::MAKE_POLYGON_NUM; ++i)
	{
		//�����������p�`�����_�ɂȂ肦�Ȃ��Ȃ�΁C���̑��p�`�͖�������
		if (candidate_polygons[i].second == ComType::EComPattern::ERROR_POS) { continue; }

		my_vec::SVector result_com;

		if (m_selecter.getComFromPolygon(candidate_polygons[i].first, candidate_polygons[i].second, &result_com))
		{
			SNode next_node = current_node;

			next_node.changeGlobalCenterOfMass(result_com, false);					//�d�S�ʒu��ύX���C����ɔ����ڒn�r�̈ʒu���ύX����

			dl_leg::changeComPattern(next_node.leg_state, candidate_polygons[i].second);		//leg_state��com_pattern��ύX����

			for (int j = 0; j < HexapodConst::LEG_NUM; ++j)
			{
				dl_leg::changeLegStateKeepTopBit(next_node.leg_state, j, 4);
			}

			next_node.changeNextNode(current_num, m_next_move);	//�[����e�m�[�h��ύX����

			if (isStable(next_node) && !isIntersectGround(next_node))
			{
				(*output_graph).push_back(next_node);
			}
		}
	}

	if (DO_DEBUG_PRINT)
	{
		std::cout << "ComMoveNodeCreatorHato::create() : " << (*output_graph).size() << std::endl;
	}
}


bool ComMoveNodeCreatorHato::isStable(const SNode& node) const
{
	//�d�S�����_�Ƃ������W�n�ŁC�r�̈ʒu���v�Z����D

	if (m_calclator.calculateStaticMargin(node) < STABLE_MARGIN)
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
		const my_vec::SVector kCoxaPos = m_calclator.getGlobalCoxaJointPos(node, i, false);	//�r�̍����̍��W(�O���[�o��)���擾����

		const float kMapTopZ = mp_map->getTopZFromDevideMap(mp_map->getDevideMapNumX(kCoxaPos.x), mp_map->getDevideMapNumY(kCoxaPos.y));

		top_z = (std::max)(top_z, kMapTopZ);	//�ł������_�����߂�
	}

	if (top_z + HexapodConst::VERTICAL_MIN_RANGE - my_math::ALLOWABLE_ERROR < node.global_center_of_mass.z)
	{
		return false;
	}

	return true;
}
