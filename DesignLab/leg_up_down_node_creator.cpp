#include "leg_up_down_node_creator.h"

#include <algorithm>
#include <iostream>

#include "com_type.h"
#include "graph_search_const.h"
#include "leg_state.h"


LegUpDownNodeCreator::LegUpDownNodeCreator(const MapState* const p_map, std::shared_ptr<AbstractHexapodStateCalculator> calc, const EHexapodMove next_move)
	: INodeCreator(p_map, calc, next_move), mp_map(p_map)
{
	if (GraphSearchConst::DO_DEBUG_PRINT)
	{
		std::cout << "[NodeCreator] LegUpDownNodeCreator : �R���X�g���N�^���Ă΂ꂽ�D\n";
	}
};


LegUpDownNodeCreator::~LegUpDownNodeCreator()
{
	if (GraphSearchConst::DO_DEBUG_PRINT)
	{
		std::cout << "[NodeCreator] LegUpDownNodeCreator : �f�X�g���N�^���Ă΂ꂽ�D\n";
	}
}


void LegUpDownNodeCreator::create(const SNode& current_node, const int current_num, std::vector<SNode>* output_graph)
{
	//�r�̗V�r�E�ڒn�ɂ���Đ�����Ƃ肤��d�S��comtype�Ƃ��Ďd�����Ă���D(�ڂ�����Comtype.h���Q��)�D�܂��͑S��true�ɂ��Ă����D
	bool is_able_type[ComType::COM_TYPE_NUM];

	for (int i = 0; i < ComType::COM_TYPE_NUM; i++)
	{
		is_able_type[i] = true;
	}

	//�d�S�����݂ǂ��ɂ��邩(�O��肩�^�񒆂�...)�Ȃǂ̃p�����[�^�͂���com pattern�Ŏd�����Ă���D(�ڂ�����Comtype.h���Q��)�D������擾����D
	int com_pattern = dl_leg::getComPatternState(current_node.leg_state);

	//com pattern���Ƃ邱�Ƃ��ł��Ȃ�com type��S��false�ɂ���D
	ComType::checkAbleComTypeFromComPattern(com_pattern, is_able_type);



	//���ɋr���n�ʂɐڒn�\�����ׂ�D

	bool is_groundable_leg[HexapodConst::LEG_NUM];			//�r���ݒu�\�Ȃ��true�ɂȂ�D���ɐڒn���Ă���Ȃ��true�ɂȂ�D
	dl_vec::SVector ground_pos[HexapodConst::LEG_NUM];	//�r���ڒn������W�D

	for (int i = 0; i < HexapodConst::LEG_NUM; i++) { ground_pos[i] = current_node.leg_pos[i]; }

	for (int i = 0; i < HexapodConst::LEG_NUM; i++)
	{
		if (dl_leg::isGrounded(current_node.leg_state, i))
		{
			//���łɐڒn���Ă���r�͐ڒn�\�Ɍ��܂��Ă���̂�true�ɂ���D
			is_groundable_leg[i] = true;
			ground_pos[i] = current_node.leg_pos[i];
		}
		else
		{
			//���ݗV�r���̋r�͎��g�̋r��ԂŐڒn�ł��邩��������D
			dl_vec::SVector res_ground_pos;

			if (isGroundableLeg(i, current_node, &res_ground_pos))
			{
				is_groundable_leg[i] = true;	//�ڒn�\�ɂ���D
				ground_pos[i] = res_ground_pos;
			}
			else
			{
				is_groundable_leg[i] = false;	//�ڒn�s�\�ɂ���D
				ComType::checkAbleComTypeFromNotGroundableLeg(i, is_able_type);	//�ڒn�s�\�ȋr�ɂ���āC�Ƃ�Ȃ�com type��S�Ă����D
			}
		}
	}


	//�q�m�[�h�𐶐�����D
	for (int i = 0; i < ComType::COM_TYPE_NUM; i++)
	{
		//���̏d�S�^�C�v���\�ł���΁C
		if (is_able_type[i])
		{
			SNode res_node = current_node;
			res_node.changeNextNode(current_num, m_next_move);

			//�V�r�E�ڒn������������D
			bool new_is_ground[HexapodConst::LEG_NUM] = {};
			ComType::getGroundLegFromComType(i, new_is_ground);

			for (int j = 0; j < HexapodConst::LEG_NUM; j++)
			{
				dl_leg::changeGround(res_node.leg_state, j, new_is_ground[j]);

				if (new_is_ground[j])
				{
					res_node.leg_pos[j] = ground_pos[j];
					res_node.leg_base_pos[j] = ground_pos[j];
				}
				else
				{
					res_node.leg_pos[j].x = 160 * HexapodConst::DEFAULT_LEG_ANGLE_COS[j];
					res_node.leg_pos[j].y = 160 * HexapodConst::DEFAULT_LEG_ANGLE_SIN[j];
					res_node.leg_pos[j].z = -10;
					res_node.leg_base_pos[j].x = res_node.leg_pos[j].x;
					res_node.leg_base_pos[j].y = res_node.leg_pos[j].y;
				}
			}

			if (m_calclator.isAblePause(res_node))
			{
				//�r���n�ʂɂ��Ă���Ȃ�Βǉ������ɑ��s�D
				(*output_graph).push_back(res_node);
			}

		}

	}

}


bool LegUpDownNodeCreator::isGroundableLeg(const int now_leg_num, const SNode& current_node, dl_vec::SVector* output_ground_pos)
{
	//for���̒���continue�ɂ��Ă� http://www9.plala.or.jp/sgwr-t/c/sec06-7.html ���Q�ƁD���Ȃ݂ɓǂ݂Â炭�Ȃ�̂Ŗ{���͎g��Ȃ��ق��������D

	if (mp_map == nullptr) { return false; }	//�}�b�v���Ȃ��Ƃ���false��Ԃ��D

	//�r���W��devide map�łǂ��ɓ����邩���ׂāC���̃}�X��2���2���͈͓̔���S�ĒT������D
	const dl_vec::SVector kGlobalLegbasePos = m_calclator.getGlobalLegBasePos(current_node, now_leg_num, false);

	int max_x_dev = mp_map->getDevideMapNumX(kGlobalLegbasePos.x) + 2;
	int min_x_dev = mp_map->getDevideMapNumX(kGlobalLegbasePos.x) - 2;
	int max_y_dev = mp_map->getDevideMapNumY(kGlobalLegbasePos.y) + 2;
	int min_y_dev = mp_map->getDevideMapNumY(kGlobalLegbasePos.y) - 2;

	////�l��devide map�͈̔͊O�ɂ���Ƃ��͊ۂ߂�D
	max_x_dev = (max_x_dev >= MapConst::LP_DIVIDE_NUM) ? MapConst::LP_DIVIDE_NUM - 1 : max_x_dev;
	min_x_dev = (min_x_dev < 0) ? 0 : min_x_dev;
	max_y_dev = (max_y_dev >= MapConst::LP_DIVIDE_NUM) ? MapConst::LP_DIVIDE_NUM - 1 : max_y_dev;
	min_y_dev = (min_y_dev < 0) ? 0 : min_y_dev;


	//devide map����S�T�����āC���݂̋r�ʒu(���U��������)�ɓK�����r�ݒu�\�_�����݂��邩���ׂ�D

	dl_vec::SVector candidate_pos;		//���݂̋r�ʒu�ɍ��v��������W�Q�D
	bool is_candidate_pos = false;		//�����W�����݂��邩�ǂ����D

	//�͈͓��̓_��S�Ē��ׂ�D
	for (int x = min_x_dev; x < max_x_dev; x++)
	{
		for (int y = min_y_dev; y < max_y_dev; y++)
		{
			const int kPosNum = mp_map->getPointNumFromDevideMap(x, y);

			for (int n = 0; n < kPosNum; n++)
			{
				dl_vec::SVector map_point_pos = mp_map->getPosFromDevideMap(x, y, n);	//�r�ݒu�\�_�̍��W�����o���D
				map_point_pos = m_calclator.convertLocalLegPos(current_node, map_point_pos, now_leg_num, false);

				//�r�ʒu���X�V�����m�[�h���쐬����D
				SNode new_node = current_node;

				new_node.leg_pos[now_leg_num] = map_point_pos;


				//�O�̌��n�_�Ɣ�r���āC���ǂ����n�_�̎��̂ݎ��s������
				if (is_candidate_pos)
				{
					//���Ε������ނ��Ă���ꍇ�͌��n�_�Ƃ��č̗p���Ȃ��D
					if (new_node.leg_base_pos[now_leg_num].projectedXY().cross(candidate_pos.projectedXY()) * new_node.leg_base_pos[now_leg_num].projectedXY().cross(map_point_pos.projectedXY()) < 0)
					{
						continue;
					}

					//���݂̋r�ʒu�ƌ��n�_�̊Ԃɏ�Q��������ꍇ�͌��n�_�Ƃ��č̗p���Ȃ��D
					if (map_point_pos.projectedXY().cross(candidate_pos.projectedXY()) * map_point_pos.projectedXY().cross(new_node.leg_base_pos[now_leg_num].projectedXY()) < 0)
					{
						continue;
					}
				}

				dl_leg::changeGround(new_node.leg_state, now_leg_num, true);

				if (!m_calclator.isLegInRange(new_node, now_leg_num)) { continue; }			//�r���͈͊O�Ȃ�Βǉ������ɑ��s�D

				//if (m_calclator.isLegInterfering(new_node)) { continue; }					//�r�������Ă���Ȃ�Βǉ������ɑ��s�D

				if (!isAbleLegPos(new_node, now_leg_num)) { continue; }	//�����W�Ƃ��āC�K���Ă��Ȃ��Ȃ�Βǉ������ɑ��s�D

				is_candidate_pos = true;
				candidate_pos = map_point_pos;
			}
		}
	}


	//���_��S�񋓂����̂��C���_������Ȃ����false
	if (!is_candidate_pos) { return false; }

	//���݂���Ȃ�C���̒��ōł��K�������̂����ʂƂ��ĕԂ��Ctrue
	(*output_ground_pos) = candidate_pos;

	return true;
}


bool LegUpDownNodeCreator::isAbleLegPos(const SNode& _node, const int _leg_num)
{
	const int _leg_state = dl_leg::getLegState(_node.leg_state, _leg_num);		//�r�ʒu���擾(1�`7)

	//�܂��ŏ��ɋr�ʒu4�̂Ƃ���ɂȂ����m���߂�D
	if ((_node.leg_base_pos[_leg_num] - _node.leg_pos[_leg_num]).lengthSquare() < dl_math::squared(LEG_MARGIN))
	{
		if (_leg_state == 4) { return true; }
		else { return false; }
	}
	else
	{
		if (_leg_state == 4) { return false; }
	}

	//�r�ʒu4�Ɣ�r���đO����납
	if (_node.leg_base_pos[_leg_num].projectedXY().cross(_node.leg_pos[_leg_num].projectedXY()) * _node.leg_pos[_leg_num].projectedXY().cross({ 1,0 }) > 0)
	{
		//�O

		if (_leg_state == 1 || _leg_state == 2 || _leg_state == 3)
		{
			return false;
		}
	}
	else
	{
		//���

		if (_leg_state == 7 || _leg_state == 6 || _leg_state == 5)
		{
			return false;
		}
	}


	//�r�ʒu4�Ɣ�r���ďォ����
	if (_leg_state == 1 || _leg_state == 5)
	{
		//�r�ʒu4�Ɣ�r���ĉ�
		if (_node.leg_base_pos[_leg_num].z - HIGH_MARGIN >= _node.leg_pos[_leg_num].z)
		{
			return true;
		}
	}
	else if (_leg_state == 3 || _leg_state == 7)
	{
		//�r�ʒu4�Ɣ�r���ď�
		if (_node.leg_base_pos[_leg_num].z + HIGH_MARGIN <= _node.leg_pos[_leg_num].z)
		{
			return true;
		}
	}
	else
	{
		//�r�ʒu4�Ɠ������炢
		if (std::abs(_node.leg_base_pos[_leg_num].z - _node.leg_pos[_leg_num].z) <= HIGH_MARGIN)
		{
			return true;
		}
	}

	return false;
}
