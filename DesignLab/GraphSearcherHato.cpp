#include "GraphSearcherHato.h"
#include "Define.h"
#include "LegState.h"

EGraphSearchResult GraphSearcherHato::searchGraphTree(const std::vector<SNode>& _graph, const STarget& _target, SNode& _output_result)
{
	// _target�̒l�ɂ���āC�T�����@��ς���K�v������D�T�����@�𒊏ۉ�����ׂ��D

	// @todo initializer�ŏ��������鏈��������

	// �^�[�Q�b�g���[�h�����i�Ɖ��肵�ď����������Ă���

	int _result_index = -1;
	float _max_move_dif = _target.TargetPosition.lengthSquare();
	int _max_leg_change = 0;

	const size_t _graph_size = _graph.size();
	size_t _parent_num = 0;

	for (size_t i = 0; i < _graph_size; i++)
	{
		if (_graph.at(i).depth == 0)
		{
			_parent_num = i;
			break;
		}
	}

	for (size_t i = 0; i < _graph_size; i++)
	{
		//�ő�[���̃m�[�h�݂̂�]������
		if (_graph.at(i).depth == Define::GRAPH_SEARCH_DEPTH)
		{
			if (_result_index < 0)
			{
				_result_index = i;
				_max_move_dif = _target.TargetPosition.lengthSquare();
				_max_leg_change = LegStateEdit::getLegUpDownCount(_graph.at(_parent_num).leg_state, _graph.at(i).leg_state);
			}

			my_vec::SVector2 _move_dif = _target.TargetPosition.projectedXY() - _graph.at(i).global_center_of_mass.projectedXY();
			int _leg_change = LegStateEdit::getLegUpDownCount(_graph.at(_parent_num).leg_state, _graph.at(i).leg_state);

			if (_max_move_dif + MARGIN_OF_MOVE > _move_dif.lengthSquare())
			{
				_max_move_dif = _move_dif.lengthSquare();
				_result_index = i;
				_max_leg_change = _leg_change;

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
