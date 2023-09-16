#include "leg_hierarchy_node_creator.h"

#include "graph_search_const.h"
#include "leg_state.h"


LegHierarchyNodeCreator::LegHierarchyNodeCreator(const MapState* const p_map, std::shared_ptr<AbstractHexapodStateCalculator> calc, const EHexapodMove next_move)
	: INodeCreator(p_map, calc, next_move),
	discrete_leg_pos_list_(
		{ EDiscreteLegPos::BACK,EDiscreteLegPos::CENTER,EDiscreteLegPos::FRONT,EDiscreteLegPos::LOWER_BACK,EDiscreteLegPos::LOWER_FRONT,
		EDiscreteLegPos::UPPER_BACK,EDiscreteLegPos::UPPER_FRONT }
	)
{
}


void LegHierarchyNodeCreator::create(const SNode& current_node, const int current_node_index, std::vector<SNode>* output_graph)
{
	//���݁C�ڒn���Ă���r�̖{���𐔂���
	const int kLiftedLegNum = dl_leg::getLiftedLegNum(current_node.leg_state);

	//�V�r���Ă���r�̖{���ɂ���ď���������
	if (kLiftedLegNum == 1)
	{
		// 1 �{�V�r���Ă���D
		create1LegLifted(current_node, current_node_index, output_graph);
	}
	else if (kLiftedLegNum == 2)
	{
		// 2 �{�V�r���Ă���D
		create2LegLifted(current_node, current_node_index, output_graph);
	}
	else if (kLiftedLegNum == 3)
	{
		// 3 �{�V�r���Ă���D
		create3LegLifted(current_node, current_node_index, output_graph);
	}
	else
	{
		//�����ɗ���̂͐ڒn���Ă���r�̐���6�{ or 1�{ or 2�{�D�n�ʂɂ��Ă���r��3�{��؂邱�Ƃ͂Ȃ��C���̂Ȃ烍�{�b�g���|��Ă��܂����߁D
		// 
		//�܂�6�{�ڒn���Ă���Ȃ�΋r�𓮂����Ȃ�(�V�r����K�v������)�D����ď������s��Ȃ��D(���̂܂܂̏�Ԃ����̃m�[�h�ɂ���D)
		SNode new_node = current_node;

		new_node.changeNextNode(current_node_index, m_next_move);		//���̃m�[�h�p�ɁC�[���E�e�E���̓�����X�V����D
		(*output_graph).emplace_back(new_node);		//�ǉ�����D
	}
}


//�S�ď�̊֐��ɂ܂Ƃ߂�Ƃ�������̂ŁC�ȉ��̊֐��ɏ����𕪂��Ă����D

void LegHierarchyNodeCreator::create1LegLifted(const SNode& current_node, const int current_node_index, std::vector<SNode>* output_graph)
{
	//�V�r���Ă���r��T���D�V�r����1�Ȃ̂�1�̐������A��͂�
	std::vector<int> lifted_leg_list;

	dl_leg::getLiftedLegIndexWithVector(current_node.leg_state, &lifted_leg_list);


	// �񋓑� EDiscreteLegPos �̑S�Ă̗v�f�Ń��[�v���񂷁D
	for (const auto i : discrete_leg_pos_list_)
	{
		SNode new_node = current_node;		//�V�����r��Ԃ𐶐�����.

		dl_leg::changeLegStateKeepTopBit(lifted_leg_list[0], i, &new_node.leg_state);	//�r��Ԃ�ύX����D

		new_node.changeNextNode(current_node_index, m_next_move);		//���̃m�[�h�p�ɁC�[���E�e�E���̓�����X�V����D

		(*output_graph).emplace_back(new_node);	//�ǉ�����D
	}
}


void LegHierarchyNodeCreator::create2LegLifted(const SNode& current_node, const int current_node_index, std::vector<SNode>* output_graph)
{
	//�V�r���Ă���r��T���D�V�r����2�Ȃ̂�2�̐������A��͂�
	std::vector<int> lifted_leg_list;

	dl_leg::getLiftedLegIndexWithVector(current_node.leg_state, &lifted_leg_list);


	// �񋓑� EDiscreteLegPos �̑S�Ă̗v�f�Ń��[�v���񂷁D
	for (const auto i : discrete_leg_pos_list_)
	{
		for (const auto j : discrete_leg_pos_list_)
		{
			SNode new_node = current_node;		//�V�����r��Ԃ𐶐�����.

			dl_leg::changeLegStateKeepTopBit(lifted_leg_list[0], i, &new_node.leg_state);			//�r��Ԃ�ύX����D
			dl_leg::changeLegStateKeepTopBit(lifted_leg_list[1], j, &new_node.leg_state);

			new_node.changeNextNode(current_node_index, m_next_move);		//���̃m�[�h�p�ɁC�[���E�e�E���̓�����X�V����D

			(*output_graph).emplace_back(new_node);	//�ǉ�����D
		}
	}
}


void LegHierarchyNodeCreator::create3LegLifted(const SNode& current_node, const int current_node_index, std::vector<SNode>* output_graph)
{
	//�V�r���Ă���r��T���D�V�r����3�Ȃ̂�3�̐������A��͂�
	std::vector<int> lifted_leg_list;

	dl_leg::getLiftedLegIndexWithVector(current_node.leg_state, &lifted_leg_list);


	// �񋓑� EDiscreteLegPos �̑S�Ă̗v�f�Ń��[�v���񂷁D
	for (const auto i : discrete_leg_pos_list_)
	{
		for (const auto j : discrete_leg_pos_list_)
		{
			for (const auto k : discrete_leg_pos_list_)
			{
				SNode new_node = current_node;		//�V�����r��Ԃ𐶐�����.

				dl_leg::changeLegStateKeepTopBit(lifted_leg_list[0], i, &new_node.leg_state);			//�r��Ԃ�ύX����D
				dl_leg::changeLegStateKeepTopBit(lifted_leg_list[1], j, &new_node.leg_state);
				dl_leg::changeLegStateKeepTopBit(lifted_leg_list[2], k, &new_node.leg_state);

				new_node.changeNextNode(current_node_index, m_next_move);		//���̃m�[�h�p�ɁC�[���E�e�E���̓�����X�V����D

				(*output_graph).push_back(new_node);	//�ǉ�����D
			}
		}
	}
}
