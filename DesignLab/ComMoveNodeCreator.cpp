#include "ComMoveNodeCreator.h"
#include "ComCandidatePolygonMaker.h"
#include "ComSelecter.h"
#include "LegState.h"

void ComMoveNodeCreator::create(const SNode& _current_node, const int _current_num, std::vector<SNode>& _output_graph)
{
	//�d�S�ړ���̌��n�_�͈̔͂��������p�`���쐬����
	ComCandidatePolygonMaker _maker;

	std::vector<std::pair<my_vec::SPolygon2, ComType::EComPattern>> _candidate_polygons;
	_maker.makeCandidatePolygon(_current_node, _candidate_polygons);

	//���͈͂�����ۂɈړ������̍��W�����肷��
	ComSelecter _selecter;

	_selecter.setCurrentNode(_current_node);

	for (const auto& i : _candidate_polygons)
	{
		my_vec::SVector _res;

		if (_selecter.getComFromPolygon(i.first, i.second, _res) == true)
		{
			SNode _next_node = _current_node;

			_next_node.changeGlobalCenterOfMass(_res);							//�d�S�ʒu��ύX���C����ɔ����ڒn�r�̈ʒu���ύX����

			LegStateEdit::changeComPattern(_next_node.leg_state, i.second);		//leg_state��com_pattern��ύX����

			_next_node.changeNextNode(_current_num, EHexapodMove::LEG_HIERARCHY_CHANGE);	//�[����e�m�[�h��ύX����

			_output_graph.push_back(_next_node);
		}
	}

	if (DO_DEBUG_PRINT)std::cout << "ComMoveNodeCreator::create() : " << _output_graph.size() << std::endl;
}