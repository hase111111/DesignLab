#include "ComMoveNodeCreatorHato.h"
#include "ComCandidatePolygonMaker.h"
#include "ComSelecterHato.h"
#include "LegState.h"

void ComMoveNodeCreatorHato::create(const SNode& _current_node, const int _current_num, std::vector<SNode>& _output_graph)
{
	std::vector<std::pair<my_vec::SPolygon2, ComType::EComPattern>> _candidate_polygons;

	//�d�S�ړ���̌��n�_�͈̔͂��������p�`���쐬����
	ComCandidatePolygonMaker _maker;
	_maker.makeCandidatePolygon(_current_node, _candidate_polygons);

	//���͈͂�����ۂɈړ������̍��W��I������
	ComSelecterHato _selecter;
	_selecter.setCurrentNode(_current_node);

	for (const auto& i : _candidate_polygons)
	{
		my_vec::SVector _res;

		if (_selecter.getComFromPolygon(i.first, i.second, _res) == true)
		{
			SNode _next_node = _current_node;

			_next_node.changeGlobalCenterOfMass(_res, false);					//�d�S�ʒu��ύX���C����ɔ����ڒn�r�̈ʒu���ύX����

			LegStateEdit::changeComPattern(_next_node.leg_state, i.second);		//leg_state��com_pattern��ύX����

			for (int i = 0; i < HexapodConst::LEG_NUM; i++) { LegStateEdit::changeLegStateKeepTopBit(_next_node.leg_state, i, 4); }

			_next_node.changeNextNode(_current_num, m_next_move);	//�[����e�m�[�h��ύX����

			_output_graph.push_back(_next_node);
		}
	}

	if (DO_DEBUG_PRINT)std::cout << "ComMoveNodeCreatorHato::create() : " << _output_graph.size() << std::endl;

	//����]�T�𖞂����Ȃ��ꍇ�͎}���肷��D
	for (auto i = _output_graph.begin(); i != _output_graph.end();)
	{
		if (isStable(*i) == false)
		{
			i = _output_graph.erase(i);
		}
		else
		{
			i++;
		}
	}

	if (DO_DEBUG_PRINT)std::cout << "ComMoveNodeCreatorHato::isStable() : " << _output_graph.size() << std::endl;

	//�n�ʂƊ����Ă���ꍇ�͎}���肷��D
	for (auto i = _output_graph.begin(); i != _output_graph.end();)
	{
		if (isIntersectGround(*i) == true)
		{
			i = _output_graph.erase(i);
		}
		else
		{
			i++;
		}
	}

	if (DO_DEBUG_PRINT)std::cout << "ComMoveNodeCreatorHato::isIntersectGround() : " << _output_graph.size() << std::endl;
}

bool ComMoveNodeCreatorHato::isStable(const SNode _node) const
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

bool ComMoveNodeCreatorHato::isIntersectGround(const SNode _node) const
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
