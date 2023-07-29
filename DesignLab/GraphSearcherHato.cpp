#include "GraphSearcherHato.h"

#include <iostream>

#include "Define.h"
#include "GraphSearchConst.h"
#include "LegState.h"


GraphSearcherHato::GraphSearcherHato()
{
	if (GraphSearchConst::DO_DEBUG_PRINT)
	{
		std::cout << "[GraphSearcher] GraphSearcherHato : �R���X�g���N�^���Ă΂ꂽ" << std::endl;
	}
}

GraphSearcherHato::~GraphSearcherHato()
{
	if (GraphSearchConst::DO_DEBUG_PRINT)
	{
		std::cout << "[GraphSearcher] GraphSearcherHato : �f�X�g���N�^���Ă΂ꂽ" << std::endl;
	}
}

EGraphSearchResult GraphSearcherHato::searchGraphTree(const std::vector<SNode>& graph, const STarget& target, SNode* output_result)
{
	if (GraphSearchConst::DO_DEBUG_PRINT)
	{
		std::cout << "[GraphSearcher] GraphSearcherHato : searchGraphTree() �T���J�n\n";
	}

	// _target�̒l�ɂ���āC�T�����@��ς���K�v������D�T�����@�𒊏ۉ�����ׂ��D

	// @todo initializer�ŏ��������鏈��������

	// �^�[�Q�b�g���[�h�����i�Ɖ��肵�ď����������Ă���

	int result_index = -1;
	float max_move_dif = target.TargetPosition.lengthSquare();
	int max_leg_change = 0;

	const size_t kGraphSize = graph.size();
	size_t parent_num = 0;

	for (size_t i = 0; i < kGraphSize; i++)
	{
		if (graph.at(i).depth == 0)
		{
			parent_num = i;
			break;
		}
	}

	for (size_t i = 0; i < kGraphSize; i++)
	{
		//�ő�[���̃m�[�h�݂̂�]������
		if (graph.at(i).depth == Define::GRAPH_SEARCH_DEPTH)
		{
			if (result_index < 0)
			{
				result_index = i;
				max_move_dif = target.TargetPosition.lengthSquare();
				max_leg_change = LegStateEdit::getLegUpDownCount(graph.at(parent_num).leg_state, graph.at(i).leg_state);
			}

			my_vec::SVector2 move_dif = target.TargetPosition.projectedXY() - graph.at(i).global_center_of_mass.projectedXY();
			int leg_change = LegStateEdit::getLegUpDownCount(graph.at(parent_num).leg_state, graph.at(i).leg_state);

			if (max_move_dif + MARGIN_OF_MOVE > move_dif.lengthSquare())
			{
				max_move_dif = move_dif.lengthSquare();
				result_index = i;
				max_leg_change = leg_change;

			}
		}
	}

	// index ���͈͊O�Ȃ�Ύ��s
	if (result_index < 0 || result_index >= kGraphSize) { return EGraphSearchResult::Failure; }

	//�[��1�܂ők���Ēl��Ԃ�
	while (graph.at(result_index).depth != 1)
	{
		result_index = graph.at(result_index).parent_num;
	}

	(*output_result) = graph.at(result_index);


	if (GraphSearchConst::DO_DEBUG_PRINT)
	{
		std::cout << "[GraphSearcher] GraphSearcherHato : searchGraphTree() �T���I��" << std::endl;
	}

	return EGraphSearchResult::Success;
}
