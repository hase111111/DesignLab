#include "GraphSearcherHato.h"
#include "Define.h"

EGraphSearchResult GraphSearcherHato::searchGraphTree(const std::vector<SNode>& _graph, const STarget& _target, SNode& _output_result)
{
	// _target�̒l�ɂ���āC�T�����@��ς���K�v������D�T�����@�𒊏ۉ�����ׂ��D

	// @todo initializer�ŏ��������鏈��������

	// �^�[�Q�b�g���[�h�����i�Ɖ��肵�ď����������Ă���

	int _result_index = -1;
	float _max_move_dif = _target.TargetPosition.lengthSquare();

	const size_t _graph_size = _graph.size();

	for (size_t i = 0; i < _graph_size; i++)
	{
		if (_result_index < 0) { _result_index = i; }

		//�ő�[���̃m�[�h�݂̂�]������
		if (_graph.at(i).depth == Define::GRAPH_SEARCH_DEPTH)
		{
			my_vec::SVector2 _move_dif = _target.TargetPosition.projectedXY() - _graph.at(i).global_center_of_mass.projectedXY();

			if (_max_move_dif > _move_dif.lengthSquare())
			{
				_max_move_dif = _move_dif.lengthSquare();
				_result_index = i;
			}
		}
	}

	// index ���͈͊O�Ȃ�Ύ��s
	if (_result_index < 0 || _result_index >= _graph_size) { return EGraphSearchResult::Failure; }

	//�[��1�܂ők���Ēl��Ԃ�
	while (_graph.at(_result_index).depth != 1)
	{
		_result_index = _graph.at(_result_index).parent_num;
	}

	_output_result = _graph.at(_result_index);

	return EGraphSearchResult::Success;
}
