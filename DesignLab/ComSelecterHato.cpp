#include "ComSelecterHato.h"
#include <algorithm>
#include <iostream>

bool ComSelecterHato::getComFromPolygon(const my_vec::SPolygon2& polygon, const ComType::EComPattern com_pattren, my_vec::SVector& output_com) const
{
	std::vector<std::pair<bool, my_vec::SVector2>> com_candidate;

	//���_�𐶐�����
	makeComCandidatePoint(polygon, &com_candidate);

	//���_�����݂̏d�S����ł������Ɉړ��ł��鏇�Ƀ\�[�g����D��3�����̂������̂̓����_���C�ȒP�Ɍ����Ɗ֐����֐��̒��Ő錾�ł����D�ނ����̂ŗ������Ȃ��Ă悢
	//�Q�l�Fhttps://qiita.com/kemkemG0/items/76988e8e62c8a2a9c90a

	//std::sort(com_candidate.begin(), com_candidate.end(),
	//	[&](const my_vec::SVector2& _v1, const my_vec::SVector2& _v2)
	//	{
	//		const my_vec::SVector2 _rotate_center = { -10000,0 };
	//		const float _rotate_r = 10000;
	//		return fabsf((_v1 - _rotate_center).length() - _rotate_r) > fabsf((_v2 - _rotate_center).length() - _rotate_r);
	//	}
	//);

	//���_�����ԂɃ`�F�b�N���C�ړ���̏d�S������]�T�𖞂����Ȃ�΁C���̓_���d�S�Ƃ��č̗p����D
	for (auto& i : com_candidate)
	{
		if (!isInMargin(polygon, i.second))
		{
			//���_�����p�`�̊O���Ȃ�Ύ��̌��_��
			i.first = false;
			continue;
		}

		//���݂̏d�S���ړ����������̂��쐬����
		SNode _temp = m_current_node;
		my_vec::SVector next_com = { i.second.x, i.second.y, m_current_node.global_center_of_mass.z };
		_temp.changeGlobalCenterOfMass(next_com, false);

		if (!m_calclator.isAllLegInRange(_temp))
		{
			//�r�����͈͊O�Ȃ�Ύ��̌��_��
			i.first = false;
			continue;
		}

		if (!m_calclator.isAblePause(_temp))
		{
			//�p��������ł��Ȃ��Ȃ�Ύ��̌��_��
			i.first = false;
			continue;
		}

		//�����܂ŗ�����C�r�����͈͓��ŁC�p��������ł���Ƃ������ƂȂ̂ŁC���̓_���d�S�Ƃ��č̗p����
		output_com = next_com;
		return true;
	}


	if (DO_DEBUG_PRINT)
	{
		std::cout << ComType::convertComPatternToBit(com_pattren) << "�̏d�S�͌�����Ȃ�����" << std::endl;
	}

	//�Y��������̂��Ȃ����false��Ԃ�
	return false;
}

void ComSelecterHato::makeComCandidatePoint(const my_vec::SPolygon2& polygon, std::vector<std::pair<bool, my_vec::SVector2>>* coms) const
{
	//�g������̏����ł͑��p�`���͂ނ悤�Ȏl�p�`�����̂ŁC�܂��͂�������
	const float min_x = polygon.getMinX();
	const float max_x = polygon.getMaxX();
	const float min_y = polygon.getMinY();
	const float max_y = polygon.getMaxY();

	const float width = max_x - min_x;
	const float height = max_y - min_y;

	const float delta_width = width / (float)DISCRETIZATION_NUM;
	const float delta_height = height / (float)DISCRETIZATION_NUM;

	//��L�̎l�p�`�̒��ɂ���_��S�Č��ɒǉ�����D
	(*coms).reserve(DISCRETIZATION_NUM * DISCRETIZATION_NUM);

	for (int x = 0; x < DISCRETIZATION_NUM; ++x)
	{
		for (int y = 0; y < DISCRETIZATION_NUM; ++y)
		{
			(*coms).push_back(std::make_pair(true, my_vec::SVector2{ min_x + delta_width * x, min_y + delta_height * y }));
		}
	}

	if (DO_DEBUG_PRINT) { std::cout << "CandidatePointNum is " << (*coms).size() << std::endl; }
}

bool ComSelecterHato::isInMargin(const my_vec::SPolygon2& polygon, const my_vec::SVector2& candidate_point) const
{
	const int k_vertex_num = polygon.getVertexNum();	//���_���D���x���g�p����̂ŁC��Ɍv�Z���Ă������ƂŌy������D

	for (int j = 0; j < k_vertex_num; ++j)
	{
		my_vec::SVector2 v1 = polygon.getVertex((j + 1) % k_vertex_num) - polygon.getVertex(j);
		v1 = v1.normalized() * -1;

		my_vec::SVector2 v_map = candidate_point - polygon.getVertex(j);

		if (v_map.cross(v1) > -STABILITY_MARGIN)
		{
			//����]�T�𖞂����Ȃ��Ȃ�Ό�₩��폜����D
			return false;
		}
	}

	return true;
}
