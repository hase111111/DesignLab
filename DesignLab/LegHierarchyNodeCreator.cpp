#include "LegHierarchyNodeCreator.h"
#include "LegState.h"
#include "NodeEdit.h"

void LegHierarchyNodeCreator::create(const SNode& _current_node, const int _current_num, std::vector<SNode>& _output_graph)
{
	//���݁C�ڒn���Ă���r�̖{���𐔂���
	const int _touch_ground_leg_num = LegState::getGroundedLegNum(_current_node.leg_state);

	//�n�ʂɂ��Ă���r�̖{���ɂ���ď���������
	if (_touch_ground_leg_num == 5)
	{
		//�V�r���Ă���r��T���D�V�r����1�Ȃ̂�1�̐������A��͂�
		std::vector<int> _lifted_leg;
		LegState::getLiftedLegNumWithVector(_current_node.leg_state, _lifted_leg);

		//���͈͓��̓_���v�Z���鏈���͒��g���Ȃ������̂őS�폜�����D

		//�r��� 0001(1) ���� 0111(7)�܂� ���̃p�^�[���𐶐�����D�Ȃ����bit�͗V�r��\���D(0�Ȃ�V�r)
		for (int i = 1; i <= LegState::DISCRETE_NUM; i++)
		{
			////�V�����r��Ԃ𐶐�����
			int _new_leg_state = _current_node.leg_state;
			LegState::changeLegState(_new_leg_state, _lifted_leg.at(0), i);

			if (Define::FLAG_DO_PRUNING == true)
			{
				if (no_use_kaisou[LegState::getLegState(_new_leg_state, 0) - 1][LegState::getLegState(_new_leg_state, 1) - 1][LegState::getLegState(_new_leg_state, 2) - 1][LegState::getLegState(_new_leg_state, 3) - 1][LegState::getLegState(_new_leg_state, 4) - 1][LegState::getLegState(_new_leg_state, 5) - 1] == 0) continue;	/*int��*/
			}

			//���ʂ�������D�ړ���̊K�w���ǂ��Ȃ̂����L�^����
			_res_transition_hierarchy.push_back(_new_leg_state);
		}
	}
	else if (_touch_ground_leg_num == 4)
	{
		//�V�r���Ă���r��T���D�V�r����2�Ȃ̂�2�̐������A��͂�
		std::vector<int> _lifted_leg;
		LegState::getLiftedLegNumWithVector(_current_node.leg_state, _lifted_leg);

		//���͈͓��̓_���v�Z���鏈���͒��g���Ȃ������̂őS�폜�����D

		//�r��� 0001(1) ���� 0111(7)�܂� ���̃p�^�[���𐶐�����D�Ȃ����bit�͗V�r��\���D(0�Ȃ�V�r)
		for (int i = 1; i <= LegState::DISCRETE_NUM; i++)
		{
			for (int j = 1; j <= LegState::DISCRETE_NUM; j++)
			{
				//�V�����r��Ԃ𐶐�����
				int _new_leg_state = _current_node.leg_state;
				LegState::changeLegState(_new_leg_state, _lifted_leg.at(0), i);
				LegState::changeLegState(_new_leg_state, _lifted_leg.at(1), j);

				if (Define::FLAG_DO_PRUNING)
				{
					if (no_use_kaisou[LegState::getLegState(_new_leg_state, 0) - 1][LegState::getLegState(_new_leg_state, 1) - 1][LegState::getLegState(_new_leg_state, 2) - 1][LegState::getLegState(_new_leg_state, 3) - 1][LegState::getLegState(_new_leg_state, 4) - 1][LegState::getLegState(_new_leg_state, 5) - 1] == 0) continue;
				}

				//���ʂ�������
				_res_transition_hierarchy.push_back(_new_leg_state);	// �ړ���̊K�w���ǂ��Ȃ̂����L�^����
			}
		}
	}
	else if (_touch_ground_leg_num == 3)
	{
		//�V�r���Ă���r��T���D�V�r����3�Ȃ̂�3�̐������A��͂�
		std::vector<int> _lifted_leg;
		LegState::getLiftedLegNumWithVector(_current_node.leg_state, _lifted_leg);

		//���͈͓��̓_���v�Z���鏈���͒��g���Ȃ������̂őS�폜�����D

		//�r��� 0001(1) ���� 0111(7)�܂� ���̃p�^�[���𐶐�����D�Ȃ����bit�͗V�r��\���D(0�Ȃ�V�r)
		for (int i = 1; i <= LegState::DISCRETE_NUM; i++)
		{
			for (int j = 1; j <= LegState::DISCRETE_NUM; j++)
			{
				for (int k = 1; k <= LegState::DISCRETE_NUM; k++)
				{
					//�V�����r��Ԃ𐶐�����
					int _new_leg_state = _current_node.leg_state;
					LegState::changeLegState(_new_leg_state, _lifted_leg.at(0), i);
					LegState::changeLegState(_new_leg_state, _lifted_leg.at(1), j);
					LegState::changeLegState(_new_leg_state, _lifted_leg.at(2), k);

					if (Define::FLAG_DO_PRUNING == true)
					{
						if (no_use_kaisou[LegState::getLegState(_new_leg_state, 0) - 1][LegState::getLegState(_new_leg_state, 1) - 1][LegState::getLegState(_new_leg_state, 2) - 1][LegState::getLegState(_new_leg_state, 3) - 1][LegState::getLegState(_new_leg_state, 4) - 1][LegState::getLegState(_new_leg_state, 5) - 1] == 0) continue;

					}

					//���ʂ�������
					_res_transition_hierarchy.push_back(_new_leg_state);	// �ړ���̊K�w���ǂ��Ȃ̂����L�^����
				}
			}
		}
	}
	else
	{
		//�����ɗ���̂͐ڒn���Ă���r�̐���6�{ or 1�{ or 2�{�D�n�ʂɂ��Ă���r��3�{��؂邱�Ƃ͂Ȃ��C���̂Ȃ烍�{�b�g���|��Ă��܂����߁D
		//�܂�6�{�ڒn���Ă���Ȃ�΋r�𓮂����Ȃ�(�V�r����K�v������)�D����ď������s��Ȃ��D(���̂܂܂̏�Ԃ����̃m�[�h�ɂ���D)
		SNode _new_node = _current_node;

		//���̃m�[�h�p�ɁC�[���E�e�E���̓�����X�V����D
		_new_node.depth++;
		_new_node.parent_num = _current_num;
		_new_node.next_move = EHexapodMove::LEG_UP_DOWN;

		//�ǉ�����D
		_output_graph.push_back(_new_node);
	}
}
