#include "LegHierarchyNodeCreator.h"
#include "LegState.h"


void LegHierarchyNodeCreator::create(const SNode& _current_node, const int _current_num, std::vector<SNode>& _output_graph)
{
	//���݁C�ڒn���Ă���r�̖{���𐔂���
	const int _touch_ground_leg_num = LegStateEdit::getGroundedLegNum(_current_node.leg_state);

	//�V�r���Ă���r�̖{���ɂ���ď���������
	if (_touch_ground_leg_num == HexapodConst::LEG_NUM - 1)
	{
		// 1 �{�V�r���Ă���D
		create1LegLifted(_current_node, _current_num, _output_graph);
	}
	else if (_touch_ground_leg_num == HexapodConst::LEG_NUM - 2)
	{
		// 2 �{�V�r���Ă���D
		create2LegLifted(_current_node, _current_num, _output_graph);
	}
	else if (_touch_ground_leg_num == HexapodConst::LEG_NUM - 3)
	{
		// 3 �{�V�r���Ă���D
		create3LegLifted(_current_node, _current_num, _output_graph);
	}
	else
	{
		//�����ɗ���̂͐ڒn���Ă���r�̐���6�{ or 1�{ or 2�{�D�n�ʂɂ��Ă���r��3�{��؂邱�Ƃ͂Ȃ��C���̂Ȃ烍�{�b�g���|��Ă��܂����߁D
		//�܂�6�{�ڒn���Ă���Ȃ�΋r�𓮂����Ȃ�(�V�r����K�v������)�D����ď������s��Ȃ��D(���̂܂܂̏�Ԃ����̃m�[�h�ɂ���D)
		SNode _new_node = _current_node;

		_new_node.changeNextNode(_current_num, m_next_move);		//���̃m�[�h�p�ɁC�[���E�e�E���̓�����X�V����D
		_output_graph.push_back(_new_node);		//�ǉ�����D
	}
}


void LegHierarchyNodeCreator::create1LegLifted(const SNode& _current_node, const int _current_num, std::vector<SNode>& _output_graph)
{
	//�V�r���Ă���r��T���D�V�r����1�Ȃ̂�1�̐������A��͂�
	std::vector<int> _lifted_leg;
	LegStateEdit::getLiftedLegNumWithVector(_current_node.leg_state, _lifted_leg);

	//�r��� 0001(1) ���� 0111(7)�܂� ���̃p�^�[���𐶐�����D�Ȃ����bit�͗V�r��\���D(0�Ȃ�V�r)
	for (int i = 1; i <= LegStateEdit::DISCRETE_NUM; i++)
	{
		SNode _new_node = _current_node;		//�V�����r��Ԃ𐶐�����.

		LegStateEdit::changeLegState(_new_node.leg_state, _lifted_leg.at(0), i);	//�r��Ԃ�ύX����D

		_new_node.changeNextNode(_current_num, m_next_move);		//���̃m�[�h�p�ɁC�[���E�e�E���̓�����X�V����D

		_output_graph.push_back(_new_node);	//�ǉ�����D
	}
}

void LegHierarchyNodeCreator::create2LegLifted(const SNode& _current_node, const int _current_num, std::vector<SNode>& _output_graph)
{
	//�V�r���Ă���r��T���D�V�r����2�Ȃ̂�2�̐������A��͂�
	std::vector<int> _lifted_leg;
	LegStateEdit::getLiftedLegNumWithVector(_current_node.leg_state, _lifted_leg);

	//�r��� 0001(1) ���� 0111(7)�܂� ���̃p�^�[���𐶐�����D�Ȃ����bit�͗V�r��\���D(0�Ȃ�V�r)
	for (int i = 1; i <= LegStateEdit::DISCRETE_NUM; i++)
	{
		for (int j = 1; j <= LegStateEdit::DISCRETE_NUM; j++)
		{
			SNode _new_node = _current_node;		//�V�����r��Ԃ𐶐�����.

			LegStateEdit::changeLegState(_new_node.leg_state, _lifted_leg.at(0), i);			//�r��Ԃ�ύX����D
			LegStateEdit::changeLegState(_new_node.leg_state, _lifted_leg.at(1), j);

			_new_node.changeNextNode(_current_num, m_next_move);		//���̃m�[�h�p�ɁC�[���E�e�E���̓�����X�V����D

			_output_graph.push_back(_new_node);	//�ǉ�����D
		}
	}
}

void LegHierarchyNodeCreator::create3LegLifted(const SNode& _current_node, const int _current_num, std::vector<SNode>& _output_graph)
{
	//�V�r���Ă���r��T���D�V�r����3�Ȃ̂�3�̐������A��͂�
	std::vector<int> _lifted_leg;
	LegStateEdit::getLiftedLegNumWithVector(_current_node.leg_state, _lifted_leg);

	//�r��� 0001(1) ���� 0111(7)�܂� ���̃p�^�[���𐶐�����D�Ȃ����bit�͗V�r��\���D(0�Ȃ�V�r)
	for (int i = 1; i <= LegStateEdit::DISCRETE_NUM; i++)
	{
		for (int j = 1; j <= LegStateEdit::DISCRETE_NUM; j++)
		{
			for (int k = 1; k <= LegStateEdit::DISCRETE_NUM; k++)
			{
				SNode _new_node = _current_node;		//�V�����r��Ԃ𐶐�����.

				LegStateEdit::changeLegState(_new_node.leg_state, _lifted_leg.at(0), i);	//�r��Ԃ�ύX����D
				LegStateEdit::changeLegState(_new_node.leg_state, _lifted_leg.at(1), j);
				LegStateEdit::changeLegState(_new_node.leg_state, _lifted_leg.at(2), k);

				_new_node.changeNextNode(_current_num, m_next_move);		//���̃m�[�h�p�ɁC�[���E�e�E���̓�����X�V����D

				_output_graph.push_back(_new_node);	//�ǉ�����D
			}
		}
	}
}
