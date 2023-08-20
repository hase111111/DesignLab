#include "com_selecter_hato.h"

#include <algorithm>
#include <iostream>

#include "leg_state.h"


bool ComSelecterHato::getComFromPolygon(const dl_vec::SPolygon2& polygon, const ComType::EComPattern com_pattren, dl_vec::SVector* output_com) const
{
	std::pair<bool, dl_vec::SVector2> com_candidate[DISCRETIZATION_NUM * DISCRETIZATION_NUM];

	//���_�𐶐�����
	if (!makeComCandidatePoint(polygon, com_candidate))
	{
		return false;
	}

	//���_���玟�̒��_�֌������ӂ𐳋K�������x�N�g�����쐬����
	std::vector<dl_vec::SVector2> edge_vec;
	edge_vec.resize(polygon.getVertexNum());

	for (int i = 0; i < polygon.getVertexNum(); ++i)
	{
		dl_vec::SVector2 edge = polygon.getVertex(i) - polygon.getVertex((i + 1) % polygon.getVertexNum());
		edge.normalized();
		edge_vec[i] = edge;
	}

	//���_�����ԂɃ`�F�b�N���C�ړ���̏d�S������]�T�𖞂����Ȃ�΁C���̓_���d�S�Ƃ��č̗p����D
	dl_vec::SVector after_move_com;
	dl_vec::SVector after_move_leg_pos[HexapodConst::LEG_NUM];

	for (int i = 0; i < DISCRETIZATION_NUM * DISCRETIZATION_NUM; ++i)
	{
		if (!isInMargin(polygon, edge_vec, com_candidate[i].second))
		{
			//���_�����p�`�̊O���Ȃ�Ύ��̌��_��
			com_candidate[i].first = false;
			continue;
		}

		//���݂̏d�S���ړ����������̂��쐬����
		after_move_com = { com_candidate[i].second.x, com_candidate[i].second.y, getCurrentNode().global_center_of_mass.z };

		for (int j = 0; j < HexapodConst::LEG_NUM; j++)
		{
			if (dl_leg::isGrounded(getCurrentNode().leg_state, j))
			{
				after_move_leg_pos[j] = getCurrentNode().leg_pos[j] - (after_move_com - getCurrentNode().global_center_of_mass);

				if (!m_calclator.isLegInRange(after_move_leg_pos[j], j))
				{
					//�r�����͈͊O�Ȃ�Ύ��̌��_��
					com_candidate[i].first = false;
					continue;
				}
			}
		}

		//if (!m_calclator.isAblePause(_temp))
		//{
		//	//�p��������ł��Ȃ��Ȃ�Ύ��̌��_��
		//	com_candidate[i].first = false;
		//	continue;
		//}
	}

	//���_�����݂̏d�S����ł������Ɉړ��ł��鏇�Ƀ\�[�g����D��3�����̂������̂̓����_���C�ȒP�Ɍ����Ɗ֐����֐��̒��Ő錾�ł����D�ނ����̂ŗ������Ȃ��Ă悢
	//�Q�l�Fhttps://qiita.com/kemkemG0/items/76988e8e62c8a2a9c90a

	const dl_vec::SVector2 k_rotate_center = { -10000,0 };
	const float k_rotate_r = 10000;

	float min_dist = -100000;
	int min_index = -1;

	for (int i = 0; i < DISCRETIZATION_NUM * DISCRETIZATION_NUM; ++i)
	{
		if (com_candidate[i].first)
		{
			const float dist = fabsf((com_candidate[i].second - k_rotate_center).length() - k_rotate_r);

			if (min_dist < dist)
			{
				min_dist = dist;
				min_index = i;
			}
		}
	}

	if (min_index == -1)
	{
		return false;
	}

	//�Y��������̂��Ȃ����false��Ԃ�
	(*output_com).x = com_candidate[min_index].second.x;
	(*output_com).y = com_candidate[min_index].second.y;
	(*output_com).z = getCurrentNode().global_center_of_mass.z;
	return true;
}


bool ComSelecterHato::makeComCandidatePoint(const dl_vec::SPolygon2& polygon, std::pair<bool, dl_vec::SVector2> coms[DISCRETIZATION_NUM * DISCRETIZATION_NUM]) const
{
	//�g������̏����ł͑��p�`���͂ނ悤�Ȏl�p�`�����̂ŁC�܂��͂�������
	const float kMinX = polygon.getMinX();
	const float kMaxX = polygon.getMaxX();
	const float kMinY = polygon.getMinY();
	const float kMaxY = polygon.getMaxY();

	const float kWidth = kMaxX - kMinX;
	const float kHeight = kMaxY - kMinY;

	if (dl_math::isEqual(kWidth, 0.0f) || dl_math::isEqual(kHeight, 0.0f)) { return false; }

	const float kDeltaWidth = kWidth / (float)DISCRETIZATION_NUM;
	const float kDeltaHeight = kHeight / (float)DISCRETIZATION_NUM;

	//��L�̎l�p�`�̒��ɂ���_��S�Č��ɒǉ�����D
	for (int x = 0; x < DISCRETIZATION_NUM; ++x)
	{
		for (int y = 0; y < DISCRETIZATION_NUM; ++y)
		{
			coms[x * DISCRETIZATION_NUM + y].first = true;
			coms[x * DISCRETIZATION_NUM + y].second.x = kMinX + kDeltaWidth * x;
			coms[x * DISCRETIZATION_NUM + y].second.y = kMinY + kDeltaHeight * y;
		}
	}

	return true;
}


bool ComSelecterHato::isInMargin(const dl_vec::SPolygon2& polygon, const std::vector<dl_vec::SVector2>& edge_vec, const dl_vec::SVector2& candidate_point) const
{
	for (int i = 0; i < polygon.getVertexNum(); ++i)
	{
		dl_vec::SVector2 v_map = candidate_point - polygon.getVertex(i);

		if (v_map.cross(edge_vec[i]) > -STABILITY_MARGIN)
		{
			//����]�T�𖞂����Ȃ��Ȃ�Ό�₩��폜����D
			return false;
		}
	}

	return true;
}
