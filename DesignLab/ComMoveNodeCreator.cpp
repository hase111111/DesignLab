#include "ComMoveNodeCreator.h"
#include "ComCandidatePolygonMaker.h"

void ComMoveNodeCreator::create(const SNode& _current_node, const int _current_num, std::vector<SNode>& _output_graph)
{
	//�d�S�ړ���̌��n�_�͈̔͂��������p�`���쐬����
	ComCandidatePolygonMaker _maker;

	std::vector<my_vec::SPolygon2> _candidate_polygons;
	_maker.makeCandidatePolygon(_current_node, _candidate_polygons);

	//���͈͂�����ۂɈړ������̍��W�����肷��
}

SNode ComMoveNodeCreator::makeNextNode(const SNode& _current_node, const int _current_num, const my_vec::SVector _next_com_pos, const ComType::EComPattern _com_pattern)
{
	return SNode();
}
