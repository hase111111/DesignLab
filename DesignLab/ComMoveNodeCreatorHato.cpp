#include "ComMoveNodeCreatorHato.h"
#include "LegState.h"

void ComMoveNodeCreatorHato::create(const SNode& _current_node, const int _current_num, std::vector<SNode>& _output_graph)
{
	std::pair<my_vec::SPolygon2, ComType::EComPattern> candidate_polygons[ComCandidatePolygonMaker::MAKE_POLYGON_NUM];

	//�d�S�ړ���̌��n�_�͈̔͂��������p�`���쐬����
	m_maker.makeCandidatePolygon(_current_node, candidate_polygons);

	//���͈͂�����ۂɈړ������̍��W��I������
	m_selecter.setCurrentNode(_current_node);

	for (int i = 0; i < ComCandidatePolygonMaker::MAKE_POLYGON_NUM; ++i)
	{
		//�����������p�`�����_�ɂȂ肦�Ȃ��Ȃ�΁C���̑��p�`�͖�������
		if (candidate_polygons[i].second == ComType::EComPattern::Error) { continue; }

		//���l�ɋr�̐ڒn��ԓI�Ɍ��ɂȂ�Ȃ��ꍇ����������D
		//bool is_leg_ground[HexapodConst::LEG_NUM];
		//for (int j = 0; j < HexapodConst::LEG_NUM; ++j) { is_leg_ground[j] = LegStateEdit::isGrounded(_current_node.leg_state, j); }
		//int com_type = ComType::convertComPatternToBit(candidate_polygons->second);

		//if (!ComType::isAbleCoM(com_type, is_leg_ground)) { continue; }

		my_vec::SVector _res;

		if (m_selecter.getComFromPolygon(candidate_polygons[i].first, candidate_polygons[i].second, _res) == true)
		{
			SNode _next_node = _current_node;

			_next_node.changeGlobalCenterOfMass(_res, false);					//�d�S�ʒu��ύX���C����ɔ����ڒn�r�̈ʒu���ύX����

			LegStateEdit::changeComPattern(_next_node.leg_state, candidate_polygons[i].second);		//leg_state��com_pattern��ύX����

			for (int j = 0; j < HexapodConst::LEG_NUM; ++j) { LegStateEdit::changeLegStateKeepTopBit(_next_node.leg_state, j, 4); }

			_next_node.changeNextNode(_current_num, m_next_move);	//�[����e�m�[�h��ύX����

			if (isStable(_next_node) && !isIntersectGround(_next_node))
			{
				_output_graph.push_back(_next_node);
			}
		}
	}

	if (DO_DEBUG_PRINT)std::cout << "ComMoveNodeCreatorHato::create() : " << _output_graph.size() << std::endl;
}

bool ComMoveNodeCreatorHato::isStable(const SNode& _node) const
{
	//�d�S�����_�Ƃ������W�n�ŁC�r�̈ʒu���v�Z����D

	if (m_Calclator.calculateStaticMargin(_node) < STABLE_MARGIN)
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
		my_vec::SVector _coxa = m_Calclator.getGlobalCoxaJointPos(_node, i, false);
		float _map_top_z = mp_Map->getTopZFromDevideMap(mp_Map->getDevideMapNumX(_coxa.x), mp_Map->getDevideMapNumY(_coxa.y));

		_top_z = (std::max)(_top_z, _map_top_z);
	}

	if (_top_z + HexapodConst::VERTICAL_MIN_RANGE - my_math::ALLOWABLE_ERROR < _node.global_center_of_mass.z)
	{
		return false;
	}

	return true;
}
