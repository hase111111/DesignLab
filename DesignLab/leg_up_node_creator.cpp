#include "leg_up_node_creator.h"

#include "com_type.h"
#include "leg_state.h"


void LegUpNodeCreator::create(const SNode& current_node, const int current_num, std::vector<SNode>* output_graph)
{
	//�r�̗V�r�E�ڒn�ɂ���Đ�����Ƃ肤��d�S��comtype�Ƃ��Ďd�����Ă���D(�ڂ�����Comtype.h���Q��)�D�܂��͑S��true�ɂ��Ă����D
	bool is_able_type[dl_com::COM_TYPE_NUM];

	for (int i = 0; i < dl_com::COM_TYPE_NUM; i++)
	{
		is_able_type[i] = true;
	}

	//�d�S�����݂ǂ��ɂ��邩(�O��肩�^�񒆂�...)�Ȃǂ̃p�����[�^�͂���com pattern�Ŏd�����Ă���D(�ڂ�����Comtype.h���Q��)�D������擾����D
	EDiscreteComPos com_pattern = dl_leg::getComPatternState(current_node.leg_state);

	//com pattern���Ƃ邱�Ƃ��ł��Ȃ�com type��S��false�ɂ���D
	dl_com::checkAbleComTypeFromComPattern(static_cast<int>(com_pattern) - 1, is_able_type);

	//���ɗV�r�̋r��ڒn���邱�Ƃ͂ł��Ȃ��D
	{
		std::vector<int> lifted_leg_num;
		dl_leg::getLiftedLegIndexWithVector(current_node.leg_state, &lifted_leg_num);

		for (auto& i : lifted_leg_num)
		{
			dl_com::checkAbleComTypeFromNotGroundableLeg(i, is_able_type);
		}
	}

	//�q�m�[�h�𐶐�����D
	for (int i = 0; i < dl_com::COM_TYPE_NUM; i++)
	{
		//���̏d�S�^�C�v���\�ł���΁C
		if (is_able_type[i])
		{
			SNode res_node = current_node;
			res_node.changeNextNode(current_num, m_next_move);

			//�V�r�E�ڒn������������D
			bool new_ground[HexapodConst::LEG_NUM] = {};
			dl_com::getGroundLegFromComType(i, new_ground);

			for (int j = 0; j < HexapodConst::LEG_NUM; j++)
			{
				dl_leg::changeGround(j, new_ground[j], &res_node.leg_state);

				if (!new_ground[j])
				{
					res_node.leg_pos[j].x = 160 * HexapodConst::DEFAULT_LEG_ANGLE_COS[j];
					res_node.leg_pos[j].y = 160 * HexapodConst::DEFAULT_LEG_ANGLE_SIN[j];
					res_node.leg_pos[j].z = -10;
				}
			}

			//�\�Ȏp���Ȃ�΁C�q�m�[�h�Ƃ��Ēǉ�����D
			if (m_calculator.isAblePause(res_node))
			{
				(*output_graph).push_back(res_node);
			}
		}
	}

	//�o�͂��ꂽ�m�[�h���Ȃ��Ȃ�΁C���̂܂܂̃m�[�h���o�͂���D
	if ((*output_graph).size() == 0)
	{
		SNode res_node = current_node;

		res_node.changeNextNode(current_num, m_next_move);

		(*output_graph).push_back(res_node);
	}

}
