#include "com_selecter.h"

#include <algorithm>
#include <iostream>


bool ComSelecter::getComFromPolygon(const dl_vec::SPolygon2& polygon, const ComType::EComPattern com_pattren, dl_vec::SVector& output_com) const
{
	std::vector<dl_vec::SVector2> coms;

	//���_�𐶐�����
	makeComCandidatePoint(polygon, coms);

	//���_�����݂̏d�S����ł������Ɉړ��ł��鏇�Ƀ\�[�g����D��3�����̂������̂̓����_���C�ȒP�Ɍ����Ɗ֐����֐��̒��Ő錾�ł����D�ނ����̂ŗ������Ȃ��Ă悢
	//�Q�l�Fhttps://qiita.com/kemkemG0/items/76988e8e62c8a2a9c90a

	std::sort(coms.begin(), coms.end(),
		[&](const dl_vec::SVector2& _v1, const dl_vec::SVector2& _v2)
		{
			return (_v1 - m_current_node.global_center_of_mass.projectedXY()).lengthSquare() > (_v2 - m_current_node.global_center_of_mass.projectedXY()).lengthSquare();
		}
	);

	//���_�����ԂɃ`�F�b�N���C�ړ���̏d�S������]�T�𖞂����Ȃ�΁C���̓_���d�S�Ƃ��č̗p����D
	for (const auto& i : coms)
	{
		//���݂̏d�S���ړ����������̂��쐬����
		SNode com_change_node = m_current_node;
		dl_vec::SVector next_com = { i.x ,i.y,m_current_node.global_center_of_mass.z };
		com_change_node.changeGlobalCenterOfMass(next_com, false);

		//if (isInMargin(polygon, i) == false) { continue; }	//����]�T�𖞂����Ȃ���Ύ��̌��_��

		if (m_calclator.isLegInterfering(com_change_node)) { continue; }	//�r�������Ă���Ύ��̌��_��

		if (!m_calclator.isAllLegInRange(com_change_node)) { continue; }	//�r�����͈͊O�Ȃ�Ύ��̌��_��

		if (!m_calclator.isAblePause(com_change_node)) { continue; }		//�p��������ł��Ȃ��Ȃ�Ύ��̌��_��

		//�����܂ŗ�����C����]�T�𖞂����C�r�������Ă��炸�C�r�����͈͓��ŁC�p��������ł���Ƃ������ƂȂ̂ŁC���̓_���d�S�Ƃ��č̗p����
		output_com = next_com;
		return true;
	}


	if (DO_DEBUG_PRINT)
	{
		std::cout << ComType::convertComPatternToBit(com_pattren) << "�̏d�S�����炸" << std::endl;
	}

	//�Y��������̂��Ȃ����false��Ԃ�
	return false;
}


void ComSelecter::makeComCandidatePoint(const dl_vec::SPolygon2& polygon, std::vector<dl_vec::SVector2>& coms) const
{
	//�g������̏����ł͑��p�`���͂ނ悤�Ȏl�p�`�����̂ŁC�܂��͂�������
	const float kMinX = polygon.getMinX();
	const float kMaxX = polygon.getMaxX();
	const float kMinY = polygon.getMinY();
	const float kMaxY = polygon.getMaxY();

	const float kWidth = kMaxX - kMinX;
	const float kHeight = kMaxY - kMinY;

	const float kDeltaWidth = kWidth / (float)DISCRETIZATION_NUM;
	const float kDeltaHeight = kHeight / (float)DISCRETIZATION_NUM;

	//��L�̎l�p�`�̒��ɂ���_��S�Č��ɒǉ�����D�������C���p�`�̒��ɂ���_�̂݁D
	for (int x = 0; x <= DISCRETIZATION_NUM; ++x)
	{
		for (int y = 0; y <= DISCRETIZATION_NUM; ++y)
		{
			const dl_vec::SVector2 kComPoint{kMinX + kDeltaWidth * x, kMinY + kDeltaHeight * y};

			if (polygon.isInside(kComPoint))
			{
				coms.push_back(kComPoint);
			}
		}
	}

	if (DO_DEBUG_PRINT) { std::cout << "CandidatePointNum is " << coms.size() << std::endl; }
}


bool ComSelecter::isInMargin(const dl_vec::SPolygon2& polygon, const dl_vec::SVector2& com) const
{
	// @todo ���삪���������̂ŏC�����邱��

	const int kVertexNum = polygon.getVertexNum();	//���_���D���x���g�p����̂ŁC��Ɍv�Z���Ă������ƂŌy������D

	for (int j = 0; j < kVertexNum; ++j)
	{
		dl_vec::SVector2 v1 = polygon.getVertex((j + 1) % kVertexNum) - polygon.getVertex(j);
		v1 = v1.normalized();

		dl_vec::SVector2 v_map = com - polygon.getVertex(j);

		if (v_map.cross(v1) < STABILITY_MARGIN)
		{
			//����]�T�𖞂����Ȃ��Ȃ�Ό�₩��폜����D
			return false;
		}
	}

	return true;
}
