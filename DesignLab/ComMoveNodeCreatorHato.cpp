#include "ComMoveNodeCreatorHato.h"
#include <iostream>
#include "GraphSearchConst.h"
#include "LegState.h"


ComMoveNodeCreatorHato::ComMoveNodeCreatorHato(const MapState* const _p_map, const EHexapodMove _next_move) : INodeCreator(_p_map, _next_move), mp_map(_p_map)
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
		if (candidate_polygons[i].second == ComType::EComPattern::error) { continue; }

		my_vec::SVector result_com;

		if (m_selecter.getComFromPolygon(candidate_polygons[i].first, candidate_polygons[i].second, &result_com) == true)
		{
			SNode next_node = current_node;

			next_node.changeGlobalCenterOfMass(result_com, false);					//�d�S�ʒu��ύX���C����ɔ����ڒn�r�̈ʒu���ύX����

			LegStateEdit::changeComPattern(next_node.leg_state, candidate_polygons[i].second);		//leg_state��com_pattern��ύX����

			for (int j = 0; j < HexapodConst::LEG_NUM; ++j)
			{
				LegStateEdit::changeLegStateKeepTopBit(next_node.leg_state, j, 4);
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

bool ComMoveNodeCreatorHato::isStable(const SNode& _node) const
{
	//�d�S�����_�Ƃ������W�n�ŁC�r�̈ʒu���v�Z����D

	if (m_calclator.calculateStaticMargin(_node) < STABLE_MARGIN)
	{
		return false;
	}
	else
	{
		return true;
	}
}

bool ComMoveNodeCreatorHato::isIntersectGround(const SNode& _node) const
{
	float _top_z = -10000.0f;	//�n�ʂƂ̌�_�̂����ł��������̂��i�[����

	for (int i = 0; i < HexapodConst::LEG_NUM; i++)
	{
		my_vec::SVector _coxa = m_calclator.getGlobalCoxaJointPos(_node, i, false);
		float _map_top_z = mp_map->getTopZFromDevideMap(mp_map->getDevideMapNumX(_coxa.x), mp_map->getDevideMapNumY(_coxa.y));

		_top_z = (std::max)(_top_z, _map_top_z);
	}

	if (_top_z + HexapodConst::VERTICAL_MIN_RANGE - my_math::ALLOWABLE_ERROR < _node.global_center_of_mass.z)
	{
		return false;
	}

	return true;
}
