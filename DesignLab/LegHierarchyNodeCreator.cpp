#include "LegHierarchyNodeCreator.h"

#include <iostream>

#include "GraphSearchConst.h"
#include "LegState.h"


LegHierarchyNodeCreator::LegHierarchyNodeCreator(const MapState* const p_map, const EHexapodMove next_move) : INodeCreator(p_map, next_move)
{
	if (GraphSearchConst::DO_DEBUG_PRINT)
	{
		std::cout << "[NodeCreator] LegHierarchyNodeCreator : �R���X�g���N�^���Ă΂ꂽ\n";
	}
}

LegHierarchyNodeCreator::~LegHierarchyNodeCreator()
{
	if (GraphSearchConst::DO_DEBUG_PRINT)
	{
		std::cout << "[NodeCreator] LegHierarchyNodeCreator : �f�X�g���N�^���Ă΂ꂽ\n";
	}
}

void LegHierarchyNodeCreator::create(const SNode& current_node, const int current_node_index, std::vector<SNode>* output_graph)
{
	//���݁C�ڒn���Ă���r�̖{���𐔂���
	const int lifted_leg_num = LegStateEdit::getLiftedLegNum(current_node.leg_state);

	//�V�r���Ă���r�̖{���ɂ���ď���������
	if (lifted_leg_num == 1)
	{
		// 1 �{�V�r���Ă���D
		create1LegLifted(current_node, current_node_index, output_graph);
	}
	else if (lifted_leg_num == 2)
	{
		// 2 �{�V�r���Ă���D
		create2LegLifted(current_node, current_node_index, output_graph);
	}
	else if (lifted_leg_num == 3)
	{
		// 3 �{�V�r���Ă���D
		create3LegLifted(current_node, current_node_index, output_graph);
	}
	else
	{
		//�����ɗ���̂͐ڒn���Ă���r�̐���6�{ or 1�{ or 2�{�D�n�ʂɂ��Ă���r��3�{��؂邱�Ƃ͂Ȃ��C���̂Ȃ烍�{�b�g���|��Ă��܂����߁D
		//�܂�6�{�ڒn���Ă���Ȃ�΋r�𓮂����Ȃ�(�V�r����K�v������)�D����ď������s��Ȃ��D(���̂܂܂̏�Ԃ����̃m�[�h�ɂ���D)
		SNode new_node = current_node;

		new_node.changeNextNode(current_node_index, m_next_move);		//���̃m�[�h�p�ɁC�[���E�e�E���̓�����X�V����D
		(*output_graph).emplace_back(new_node);		//�ǉ�����D
	}
}


void LegHierarchyNodeCreator::create1LegLifted(const SNode& current_node, const int current_node_index, std::vector<SNode>* output_graph)
{
	//�V�r���Ă���r��T���D�V�r����1�Ȃ̂�1�̐������A��͂�
	std::vector<int> lifted_leg_list;
	LegStateEdit::getLiftedLegNumWithVector(current_node.leg_state, lifted_leg_list);

	//�r��� 0001(1) ���� 0111(7)�܂� ���̃p�^�[���𐶐�����D�Ȃ����bit�͗V�r��\���D(0�Ȃ�V�r)
	for (int i = 1; i <= LegStateEdit::DISCRETE_NUM; i++)
	{
		SNode new_node = current_node;		//�V�����r��Ԃ𐶐�����.

		LegStateEdit::changeLegState(new_node.leg_state, lifted_leg_list.at(0), i);	//�r��Ԃ�ύX����D

		new_node.changeNextNode(current_node_index, m_next_move);		//���̃m�[�h�p�ɁC�[���E�e�E���̓�����X�V����D

		(*output_graph).emplace_back(new_node);	//�ǉ�����D
	}
}

void LegHierarchyNodeCreator::create2LegLifted(const SNode& current_node, const int current_node_index, std::vector<SNode>* output_graph)
{
	//�V�r���Ă���r��T���D�V�r����2�Ȃ̂�2�̐������A��͂�
	std::vector<int> lifted_leg_list;
	LegStateEdit::getLiftedLegNumWithVector(current_node.leg_state, lifted_leg_list);

	//�r��� 0001(1) ���� 0111(7)�܂� ���̃p�^�[���𐶐�����D�Ȃ����bit�͗V�r��\���D(0�Ȃ�V�r)
	for (int i = 1; i <= LegStateEdit::DISCRETE_NUM; i++)
	{
		for (int j = 1; j <= LegStateEdit::DISCRETE_NUM; j++)
		{
			SNode new_node = current_node;		//�V�����r��Ԃ𐶐�����.

			LegStateEdit::changeLegState(new_node.leg_state, lifted_leg_list.at(0), i);			//�r��Ԃ�ύX����D
			LegStateEdit::changeLegState(new_node.leg_state, lifted_leg_list.at(1), j);

			new_node.changeNextNode(current_node_index, m_next_move);		//���̃m�[�h�p�ɁC�[���E�e�E���̓�����X�V����D

			(*output_graph).emplace_back(new_node);	//�ǉ�����D
		}
	}
}

void LegHierarchyNodeCreator::create3LegLifted(const SNode& current_node, const int current_node_index, std::vector<SNode>* output_graph)
{
	//�V�r���Ă���r��T���D�V�r����3�Ȃ̂�3�̐������A��͂�
	std::vector<int> lifted_leg_list;
	LegStateEdit::getLiftedLegNumWithVector(current_node.leg_state, lifted_leg_list);

	//�r��� 0001(1) ���� 0111(7)�܂� ���̃p�^�[���𐶐�����D�Ȃ����bit�͗V�r��\���D(0�Ȃ�V�r)
	for (int i = 1; i <= LegStateEdit::DISCRETE_NUM; i++)
	{
		for (int j = 1; j <= LegStateEdit::DISCRETE_NUM; j++)
		{
			for (int k = 1; k <= LegStateEdit::DISCRETE_NUM; k++)
			{
				SNode new_node = current_node;		//�V�����r��Ԃ𐶐�����.

				LegStateEdit::changeLegState(new_node.leg_state, lifted_leg_list.at(0), i);	//�r��Ԃ�ύX����D
				LegStateEdit::changeLegState(new_node.leg_state, lifted_leg_list.at(1), j);
				LegStateEdit::changeLegState(new_node.leg_state, lifted_leg_list.at(2), k);

				new_node.changeNextNode(current_node_index, m_next_move);		//���̃m�[�h�p�ɁC�[���E�e�E���̓�����X�V����D

				(*output_graph).push_back(new_node);	//�ǉ�����D
			}
		}
	}
}
