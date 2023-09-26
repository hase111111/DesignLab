#include "leg_up_down_node_creator.h"

#include <algorithm>

#include <boost/dynamic_bitset.hpp>

#include "com_type.h"
#include "designlab_math_util.h"
#include "graph_search_const.h"
#include "leg_state.h"

namespace dlm = designlab::math_util;


LegUpDownNodeCreator::LegUpDownNodeCreator(const DevideMapState& map, const std::shared_ptr<const AbstractHexapodStateCalculator>& calc, const HexapodMove next_move) :
	kLegMargin(20),
	kHighMargin(5),
	map_(map),
	calclator_ptr_(calc),
	next_move_(next_move)
{
};



void LegUpDownNodeCreator::Create(const SNode& current_node, const int current_num, std::vector<SNode>* output_graph)
{
	//�r�̗V�r�E�ڒn�ɂ���Đ�����Ƃ肤��d�S��comtype�Ƃ��Ďd�����Ă���D(�ڂ�����Comtype.h���Q��)�D�܂��͑S��true�ɂ��Ă����D
	boost::dynamic_bitset<> is_able_leg_ground_pattern(dl_com::getLegGroundPatternNum());

	is_able_leg_ground_pattern.set();	//�S��true�ɂ���D



	//�܂����U�����ꂽ�d�S�ʒu�����蓾�Ȃ��ڒn�p�^�[�������O����D

	dl_com::banLegGroundPatternFromCom(dl_leg::getComPatternState(current_node.leg_state), &is_able_leg_ground_pattern);



	//���ɋr���n�ʂɐڒn�\�����ׂ�D

	bool is_groundable_leg[HexapodConst::LEG_NUM];			//�r���ݒu�\�Ȃ��true�ɂȂ�D���ɐڒn���Ă���Ȃ��true�ɂȂ�D
	designlab::Vector3 ground_pos[HexapodConst::LEG_NUM];		//�r���ڒn������W�D

	for (int i = 0; i < HexapodConst::LEG_NUM; i++) { ground_pos[i] = current_node.leg_pos[i]; }

	for (int i = 0; i < HexapodConst::LEG_NUM; i++)
	{
		if (dl_leg::IsGrounded(current_node.leg_state, i))
		{
			//���łɐڒn���Ă���r�͐ڒn�\�Ɍ��܂��Ă���̂�true�ɂ���D
			is_groundable_leg[i] = true;
			ground_pos[i] = current_node.leg_pos[i];
		}
		else
		{
			//���ݗV�r���̋r�͎��g�̋r��ԂŐڒn�ł��邩��������D
			designlab::Vector3 res_ground_pos;

			if (IsGroundableLeg(i, current_node, &res_ground_pos))
			{
				is_groundable_leg[i] = true;	//�ڒn�\�ɂ���D
				ground_pos[i] = res_ground_pos;
			}
			else
			{
				is_groundable_leg[i] = false;	//�ڒn�s�\�ɂ���D
				dl_com::banLegGroundPatternFromNotGroundableLeg(i, &is_able_leg_ground_pattern);
			}
		}
	}


	//�q�m�[�h�𐶐�����D
	for (int i = 0; i < dl_com::getLegGroundPatternNum(); i++)
	{
		//���̏d�S�^�C�v���\�ł���΁C
		if (is_able_leg_ground_pattern[i])
		{
			SNode res_node = current_node;

			res_node.ChangeToNextNode(current_num, next_move_);


			//�V�r�E�ڒn������������D
			dl_leg::LegGroundedBit new_is_ground = dl_com::getLegGroundedBitFromLegGroundPatternIndex(i);

			dl_leg::changeAllLegGround(new_is_ground, &res_node.leg_state);


			//�r�ʒu������������D
			for (int j = 0; j < HexapodConst::LEG_NUM; j++)
			{
				if (new_is_ground[j])
				{
					res_node.leg_pos[j] = ground_pos[j];

					res_node.leg_reference_pos[j] = ground_pos[j];
				}
				else
				{
					res_node.leg_pos[j] = calclator_ptr_->GetFreeLegPosition(j);
					//res_node.leg_pos[j].x = 160 * HexapodConst::DEFAULT_LEG_ANGLE_COS[j];
					//res_node.leg_pos[j].y = 160 * HexapodConst::DEFAULT_LEG_ANGLE_SIN[j];
					//res_node.leg_pos[j].z = -25;

					res_node.leg_reference_pos[j].x = res_node.leg_pos[j].x;
					res_node.leg_reference_pos[j].y = res_node.leg_pos[j].y;
				}
			}

			if (calclator_ptr_->CalculateStabilityMargin(res_node.leg_state,res_node.leg_pos) >= 0)
			{
				//�r���n�ʂɂ��Ă���Ȃ�Βǉ������ɑ��s�D
				(*output_graph).push_back(res_node);
			}

		}	//if is_able_leg_ground_pattern[i]

	}	//for i

}


bool LegUpDownNodeCreator::IsGroundableLeg(const int now_leg_num, const SNode& current_node, designlab::Vector3* output_ground_pos)
{
	//for���̒���continue�ɂ��Ă� http://www9.plala.or.jp/sgwr-t/c/sec06-7.html ���Q�ƁD���Ȃ݂ɓǂ݂Â炭�Ȃ�̂Ŗ{���͎g��Ȃ��ق��������D�D

	//�r���W��devide map�łǂ��ɓ����邩���ׂāC���̃}�X��2���2���͈͓̔���S�ĒT������D
	const designlab::Vector3 kGlobalLegbasePos = calclator_ptr_->GetGlobalLegPosition(now_leg_num, current_node.leg_reference_pos[now_leg_num], current_node.global_center_of_mass, current_node.rot, false);
	//m_calclator.getGlobalLegBasePos(current_node, now_leg_num, false);

	int max_x_dev = map_.GetDevideMapIndexX(kGlobalLegbasePos.x) + 2;
	int min_x_dev = map_.GetDevideMapIndexX(kGlobalLegbasePos.x) - 2;
	int max_y_dev = map_.GetDevideMapIndexY(kGlobalLegbasePos.y) + 2;
	int min_y_dev = map_.GetDevideMapIndexY(kGlobalLegbasePos.y) - 2;

	////�l��devide map�͈̔͊O�ɂ���Ƃ��͊ۂ߂�D
	max_x_dev = (max_x_dev >= MapConst::LP_DIVIDE_NUM) ? MapConst::LP_DIVIDE_NUM - 1 : max_x_dev;
	min_x_dev = (min_x_dev < 0) ? 0 : min_x_dev;
	max_y_dev = (max_y_dev >= MapConst::LP_DIVIDE_NUM) ? MapConst::LP_DIVIDE_NUM - 1 : max_y_dev;
	min_y_dev = (min_y_dev < 0) ? 0 : min_y_dev;


	//devide map����S�T�����āC���݂̋r�ʒu(���U��������)�ɓK�����r�ݒu�\�_�����݂��邩���ׂ�D

	designlab::Vector3 candidate_pos;		//���݂̋r�ʒu�ɍ��v��������W�Q�D
	bool is_candidate_pos = false;		//�����W�����݂��邩�ǂ����D

	//�͈͓��̓_��S�Ē��ׂ�D
	for (int x = min_x_dev; x < max_x_dev; x++)
	{
		for (int y = min_y_dev; y < max_y_dev; y++)
		{
			const int kPosNum = map_.GetPointNum(x, y);

			for (int n = 0; n < kPosNum; n++)
			{
				designlab::Vector3 map_point_pos = map_.GetPointPos(x, y, n);	//�r�ݒu�\�_�̍��W�����o���D
				map_point_pos = calclator_ptr_->ConvertGlobalToLegPosition(now_leg_num, map_point_pos, current_node.global_center_of_mass, current_node.rot, false);

				//�r�ʒu���X�V�����m�[�h���쐬����D
				SNode new_node = current_node;

				new_node.leg_pos[now_leg_num] = map_point_pos;


				//�O�̌��n�_�Ɣ�r���āC���ǂ����n�_�̎��̂ݎ��s������
				if (is_candidate_pos)
				{
					//���Ε������ނ��Ă���ꍇ�͌��n�_�Ƃ��č̗p���Ȃ��D
					if (new_node.leg_reference_pos[now_leg_num].ProjectedXY().Cross(candidate_pos.ProjectedXY()) * new_node.leg_reference_pos[now_leg_num].ProjectedXY().Cross(map_point_pos.ProjectedXY()) < 0)
					{
						continue;
					}

					//���݂̋r�ʒu�ƌ��n�_�̊Ԃɏ�Q��������ꍇ�͌��n�_�Ƃ��č̗p���Ȃ��D
					if (map_point_pos.ProjectedXY().Cross(candidate_pos.ProjectedXY()) * map_point_pos.ProjectedXY().Cross(new_node.leg_reference_pos[now_leg_num].ProjectedXY()) < 0)
					{
						continue;
					}
				}

				dl_leg::changeGround(now_leg_num, true, &new_node.leg_state);

				if (!calclator_ptr_->IsLegInRange(now_leg_num, new_node.leg_pos[now_leg_num])) { continue; }			//�r���͈͊O�Ȃ�Βǉ������ɑ��s�D

				//if (m_calclator.IsLegInterfering(new_node)) { continue; }					//�r�������Ă���Ȃ�Βǉ������ɑ��s�D

				if (!IsAbleLegPos(new_node, now_leg_num)) { continue; }	//�����W�Ƃ��āC�K���Ă��Ȃ��Ȃ�Βǉ������ɑ��s�D

				is_candidate_pos = true;
				candidate_pos = map_point_pos;
			}

		}	//for y

	}	//for x


	//���_��S�񋓂����̂��C���_������Ȃ����false
	if (!is_candidate_pos) { return false; }

	//���݂���Ȃ�C���̒��ōł��K�������̂����ʂƂ��ĕԂ��Ctrue
	(*output_ground_pos) = candidate_pos;

	return true;
}


bool LegUpDownNodeCreator::IsAbleLegPos(const SNode& _node, const int leg_index)
{
	const DiscreteLegPos _leg_state = dl_leg::getLegState(_node.leg_state, leg_index);		//�r�ʒu���擾(1�`7)

	//�܂��ŏ��ɋr�ʒu4�̂Ƃ���ɂȂ����m���߂�D
	if ((_node.leg_reference_pos[leg_index] - _node.leg_pos[leg_index]).LengthSquare() < dlm::Squared(kLegMargin))
	{
		if (_leg_state == DiscreteLegPos::kCenter) { return true; }
		else { return false; }
	}
	else
	{
		if (_leg_state == DiscreteLegPos::kCenter) { return false; }
	}

	//�r�ʒu4�Ɣ�r���đO����납
	if (_node.leg_reference_pos[leg_index].ProjectedXY().Cross(_node.leg_pos[leg_index].ProjectedXY()) * _node.leg_pos[leg_index].ProjectedXY().Cross({ 1,0 }) > 0)
	{
		//�O

		if (_leg_state == DiscreteLegPos::kLowerFront || _leg_state == DiscreteLegPos::kFront || _leg_state == DiscreteLegPos::kUpperFront)
		{
			return false;
		}
	}
	else
	{
		//���

		if (_leg_state == DiscreteLegPos::kLowerBack || _leg_state == DiscreteLegPos::kBack || _leg_state == DiscreteLegPos::kUpperBack)
		{
			return false;
		}
	}


	//�r�ʒu4�Ɣ�r���ďォ����
	if (_leg_state == DiscreteLegPos::kLowerFront || _leg_state == DiscreteLegPos::kLowerBack)
	{
		//�r�ʒu4�Ɣ�r���ĉ�
		if (_node.leg_reference_pos[leg_index].z - kHighMargin >= _node.leg_pos[leg_index].z)
		{
			return true;
		}
	}
	else if (_leg_state == DiscreteLegPos::kUpperFront || _leg_state == DiscreteLegPos::kUpperBack)
	{
		//�r�ʒu4�Ɣ�r���ď�
		if (_node.leg_reference_pos[leg_index].z + kHighMargin <= _node.leg_pos[leg_index].z)
		{
			return true;
		}
	}
	else
	{
		//�r�ʒu4�Ɠ������炢
		if (std::abs(_node.leg_reference_pos[leg_index].z - _node.leg_pos[leg_index].z) <= kHighMargin)
		{
			return true;
		}
	}

	return false;
}
