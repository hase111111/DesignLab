#include "com_selecter_hato.h"

#include <algorithm>
#include <iostream>

#include "designlab_math_util.h"
#include "leg_state.h"


namespace dllf = designlab::leg_func;
namespace dlm = designlab::math_util;


bool ComSelecterHato::GetComFromPolygon(const designlab::Polygon2& polygon, designlab::Vector3* output_com) const
{
	std::pair<bool, designlab::Vector2> com_candidate[kDiscretizationNum * kDiscretizationNum];

	//���_�𐶐�����
	if (!MakeComCandidatePoint(polygon, com_candidate))
	{
		return false;
	}

	//���_���玟�̒��_�֌������ӂ𐳋K�������x�N�g�����쐬����
	std::vector<designlab::Vector2> edge_vec;
	edge_vec.resize(polygon.GetVertexNum());

	for (int i = 0; i < polygon.GetVertexNum(); ++i)
	{
		designlab::Vector2 edge = polygon.GetVertex(i) - polygon.GetVertex((i + 1) % polygon.GetVertexNum());
		edge.GetNormalized();
		edge_vec[i] = edge;
	}

	//���_�����ԂɃ`�F�b�N���C�ړ���̏d�S������]�T�𖞂����Ȃ�΁C���̓_���d�S�Ƃ��č̗p����D
	designlab::Vector3 after_move_com;
	designlab::Vector3 after_move_leg_pos[HexapodConst::kLegNum];

	for (int i = 0; i < kDiscretizationNum * kDiscretizationNum; ++i)
	{
		if (!IsInMargin(polygon, edge_vec, com_candidate[i].second))
		{
			//���_�����p�`�̊O���Ȃ�Ύ��̌��_��
			com_candidate[i].first = false;
			continue;
		}

		//���݂̏d�S���ړ����������̂��쐬����
		after_move_com = { com_candidate[i].second.x, com_candidate[i].second.y, GetCurrentNode().global_center_of_mass.z };

		for (int j = 0; j < HexapodConst::kLegNum; j++)
		{
			if (dllf::IsGrounded(GetCurrentNode().leg_state, j))
			{
				after_move_leg_pos[j] = GetCurrentNode().leg_pos[j] - (after_move_com - GetCurrentNode().global_center_of_mass);

				if (!calculator_ptr_->IsLegInRange(j, after_move_leg_pos[j]))
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

	//���_�̒����猻�݂̏d�S����ł������Ɉړ��ł�����̂�I������

	const designlab::Vector2 k_rotate_center = { -10000,0 };
	const float k_rotate_r = 10000;

	float min_dist = -100000;
	int min_index = -1;

	for (int i = 0; i < kDiscretizationNum * kDiscretizationNum; ++i)
	{
		if (com_candidate[i].first)
		{
			const float dist = fabsf((com_candidate[i].second - k_rotate_center).GetLength() - k_rotate_r);

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
	(*output_com).z = GetCurrentNode().global_center_of_mass.z;
	return true;
}


bool ComSelecterHato::MakeComCandidatePoint(const designlab::Polygon2& polygon, std::pair<bool, designlab::Vector2> coms[kDiscretizationNum * kDiscretizationNum]) const
{
	//�g������̏����ł͑��p�`���͂ނ悤�Ȏl�p�`�����̂ŁC�܂��͂�������
	const float kMinX = polygon.GetMinX();
	const float kMaxX = polygon.GetMaxX();
	const float kMinY = polygon.GetMinY();
	const float kMaxY = polygon.GetMaxY();

	const float kWidth = kMaxX - kMinX;
	const float kHeight = kMaxY - kMinY;

	if (dlm::IsEqual(kWidth, 0.0f) || dlm::IsEqual(kHeight, 0.0f)) { return false; }

	const float kDeltaWidth = kWidth / (float)kDiscretizationNum;
	const float kDeltaHeight = kHeight / (float)kDiscretizationNum;

	//��L�̎l�p�`�̒��ɂ���_��S�Č��ɒǉ�����D
	for (int x = 0; x < kDiscretizationNum; ++x)
	{
		for (int y = 0; y < kDiscretizationNum; ++y)
		{
			coms[x * kDiscretizationNum + y].first = true;
			coms[x * kDiscretizationNum + y].second.x = kMinX + kDeltaWidth * x;
			coms[x * kDiscretizationNum + y].second.y = kMinY + kDeltaHeight * y;
		}
	}

	return true;
}


bool ComSelecterHato::IsInMargin(const designlab::Polygon2& polygon, const std::vector<designlab::Vector2>& edge_vec, const designlab::Vector2& candidate_point) const
{
	for (int i = 0; i < polygon.GetVertexNum(); ++i)
	{
		designlab::Vector2 v_map = candidate_point - polygon.GetVertex(i);

		if (v_map.Cross(edge_vec[i]) > -kStabilityMargin)
		{
			//����]�T�𖞂����Ȃ��Ȃ�Ό�₩��폜����D
			return false;
		}
	}

	return true;
}
