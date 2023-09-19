//#include "leg_up_node_creator.h"
//
//#include "com_type.h"
//#include "leg_state.h"
//
//
//void LegUpNodeCreator::create(const SNode& current_node, const int current_num, std::vector<SNode>* output_graph)
//{
//
//	//�r�̗V�r�E�ڒn�ɂ���Đ�����Ƃ肤��d�S��comtype�Ƃ��Ďd�����Ă���D(�ڂ�����Comtype.h���Q��)�D�܂��͑S��true�ɂ��Ă����D
//	boost::dynamic_bitset<> is_able_leg_ground_pattern(dl_com::getLegGroundPatternNum());
//
//	is_able_leg_ground_pattern.set();	//�S��true�ɂ���D
//
//
//
//	//�܂����U�����ꂽ�d�S�ʒu�����蓾�Ȃ��ڒn�p�^�[�������O����D
//
//	dl_com::banLegGroundPatternFromCom(dl_leg::getComPatternState(current_node.leg_state), &is_able_leg_ground_pattern);
//
//
//
//	//���ɗV�r�̋r��ڒn���邱�Ƃ͂ł��Ȃ��D
//	{
//		std::vector<int> lifted_leg_num;
//		dl_leg::getLiftedLegIndexWithVector(current_node.leg_state, &lifted_leg_num);
//
//		for (auto& i : lifted_leg_num)
//		{
//			dl_com::banLegGroundPatternFromNotGroundableLeg(i, &is_able_leg_ground_pattern);
//		}
//	}
//
//
//
//	//�q�m�[�h�𐶐�����D
//	for (int i = 0; i < dl_com::getLegGroundPatternNum(); i++)
//	{
//		//���̏d�S�^�C�v���\�ł���΁C
//		if (is_able_leg_ground_pattern[i])
//		{
//			SNode res_node = current_node;
//
//			res_node.changeNextNode(current_num, m_next_move);
//
//
//			//�V�r�E�ڒn������������D
//			dl_leg::LegGroundedBit new_is_ground = dl_com::getLegGroundedBitFromLegGroundPatternIndex(i);
//
//			dl_leg::changeAllLegGround(new_is_ground, &res_node.leg_state);
//
//
//			for (int j = 0; j < HexapodConst::LEG_NUM; j++)
//			{
//				dl_leg::changeGround(j, new_is_ground[j], &res_node.leg_state);
//
//				if (!new_is_ground[j])
//				{
//					res_node.leg_pos[j].x = 160 * HexapodConst::DEFAULT_LEG_ANGLE_COS[j];
//					res_node.leg_pos[j].y = 160 * HexapodConst::DEFAULT_LEG_ANGLE_SIN[j];
//					res_node.leg_pos[j].z = -25;
//
//					res_node.leg_base_pos[j].x = 160 * HexapodConst::DEFAULT_LEG_ANGLE_COS[j];
//					res_node.leg_base_pos[j].y = 160 * HexapodConst::DEFAULT_LEG_ANGLE_SIN[j];
//				}
//			}
//
//			//�\�Ȏp���Ȃ�΁C�q�m�[�h�Ƃ��Ēǉ�����D
//			if (m_calculator.isAblePause(res_node))
//			{
//				(*output_graph).push_back(res_node);
//			}
//		}
//	}
//
//	//�o�͂��ꂽ�m�[�h���Ȃ��Ȃ�΁C���̂܂܂̃m�[�h���o�͂���D
//	if ((*output_graph).size() == 0)
//	{
//		SNode res_node = current_node;
//
//		res_node.changeNextNode(current_num, m_next_move);
//
//		(*output_graph).push_back(res_node);
//	}
//
//}
