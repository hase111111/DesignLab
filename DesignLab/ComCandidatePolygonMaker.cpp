#include "ComCandidatePolygonMaker.h"
#include "MyLine.h"
#include "HexapodConst.h"

void ComCandidatePolygonMaker::makeCandidatePolygon(const SNode& _node, std::pair<my_vec::SPolygon2, ComType::EComPattern> output_poly[MAKE_POLYGON_NUM]) const
{
	my_vec::SVector2 _leg_pos_xy[HexapodConst::LEG_NUM];	//XY���ʂɎˉe�����r�ʒu���Z�o����

	for (int i = 0; i < HexapodConst::LEG_NUM; i++)
	{
		_leg_pos_xy[i] = m_calc.getGlobalLegPos(_node, i, false).projectedXY();		//�r�ʒu(�O���[�o�����W)��XY���ʂɎˉe����
	}

	//���S���͂ނ悤��4�p�`���쐬����
	makeCandidateBox(_leg_pos_xy, 0, &output_poly[0].first); output_poly[0].second = ComType::EComPattern::left_front;
	makeCandidateBox(_leg_pos_xy, 1, &output_poly[1].first); output_poly[1].second = ComType::EComPattern::left_back;
	makeCandidateBox(_leg_pos_xy, 2, &output_poly[2].first); output_poly[2].second = ComType::EComPattern::back;
	makeCandidateBox(_leg_pos_xy, 3, &output_poly[3].first); output_poly[3].second = ComType::EComPattern::right_back;
	makeCandidateBox(_leg_pos_xy, 4, &output_poly[4].first); output_poly[4].second = ComType::EComPattern::right_front;
	makeCandidateBox(_leg_pos_xy, 5, &output_poly[5].first); output_poly[5].second = ComType::EComPattern::front;

	//���S��3�p�`���쐬����
	makeCandidateTriangle(_leg_pos_xy, &output_poly[6].first, &output_poly[6].second);

	//�����������p�`�����������ǂ������`�F�b�N���C�ُ�Ȃ��͍̂폜����
	for (int i = 0; i < MAKE_POLYGON_NUM; ++i)
	{
		if (output_poly[i].first.isConvex() == false)
		{
			output_poly[i].second = ComType::EComPattern::Error;
		}
	}

	//�S�Ă̑��p�`��cout�ŏo�͂���
	if (DO_DEBUG_PRINT)
	{
		for (int i = 0; i < MAKE_POLYGON_NUM; ++i)
		{
			std::cout << "ComCandidatePolygonMaker::makeCandidatePolygon() : " << output_poly[i].first << std::endl;
		}
	}
}

void ComCandidatePolygonMaker::makeCandidateBox(const my_vec::SVector2 leg_pos[HexapodConst::LEG_NUM], const int start_leg_num, my_vec::SPolygon2* output_poly) const
{
	//�r�ʒu����Ō��ԁD���̌�_����d�S���n�_�����݂��鑽�p�`�����߂�
	my_vec::SLine2 _leg_line_02(leg_pos[(start_leg_num + 0) % HexapodConst::LEG_NUM], leg_pos[(start_leg_num + 2) % HexapodConst::LEG_NUM]);
	my_vec::SLine2 _leg_line_03(leg_pos[(start_leg_num + 0) % HexapodConst::LEG_NUM], leg_pos[(start_leg_num + 3) % HexapodConst::LEG_NUM]);
	my_vec::SLine2 _leg_line_14(leg_pos[(start_leg_num + 1) % HexapodConst::LEG_NUM], leg_pos[(start_leg_num + 4) % HexapodConst::LEG_NUM]);
	my_vec::SLine2 _leg_line_15(leg_pos[(start_leg_num + 1) % HexapodConst::LEG_NUM], leg_pos[(start_leg_num + 5) % HexapodConst::LEG_NUM]);
	my_vec::SLine2 _leg_line_25(leg_pos[(start_leg_num + 2) % HexapodConst::LEG_NUM], leg_pos[(start_leg_num + 5) % HexapodConst::LEG_NUM]);

	//��_�����߂�
	my_vec::SVector2 _intersection_02_14 = _leg_line_02.getIntersection(_leg_line_14);
	my_vec::SVector2 _intersection_02_15 = _leg_line_02.getIntersection(_leg_line_15);
	my_vec::SVector2 _intersection_03_14 = _leg_line_03.getIntersection(_leg_line_14);
	my_vec::SVector2 _intersection_03_15 = _leg_line_03.getIntersection(_leg_line_15);

	//���S��0�Ԃ̋r�ʒu�����񂾐��������߂�
	my_vec::SLine2 _leg_line_0_center(leg_pos[(start_leg_num + 0) % HexapodConst::LEG_NUM], _intersection_03_14);

	//���p�`����

	(*output_poly).reset();

	if (_leg_line_0_center.hasIntersection(_leg_line_25) == true)
	{
		//��_������ꍇ�C5�p�`�̑��p�`���쐬����
		my_vec::SVector2 _intersection_03_25 = _leg_line_03.getIntersection(_leg_line_25);
		my_vec::SVector2 _intersection_14_25 = _leg_line_14.getIntersection(_leg_line_25);

		(*output_poly).addVertex(_intersection_03_15);
		(*output_poly).addVertex(_intersection_02_15);
		(*output_poly).addVertex(_intersection_02_14);
		(*output_poly).addVertex(_intersection_14_25);
		(*output_poly).addVertex(_intersection_03_25);
	}
	else
	{
		//��_���Ȃ��ꍇ�C���ɋ��߂�4�_�ŁC4�p�`�̑��p�`���쐬����
		(*output_poly).addVertex(_intersection_03_15);
		(*output_poly).addVertex(_intersection_02_15);
		(*output_poly).addVertex(_intersection_02_14);
		(*output_poly).addVertex(_intersection_03_14);
	}
}

void ComCandidatePolygonMaker::makeCandidateTriangle(const my_vec::SVector2 leg_pos[HexapodConst::LEG_NUM], my_vec::SPolygon2* out_poly, ComType::EComPattern* out_com_pattern) const
{
	using my_vec::SLine2;
	using my_vec::SPolygon2;
	using my_vec::SVector2;

	SLine2 _leg_line_03(leg_pos[0], leg_pos[3]);
	SLine2 _leg_line_14(leg_pos[1], leg_pos[4]);
	SLine2 _leg_line_25(leg_pos[2], leg_pos[5]);

	//��_�����߂�
	SVector2 _intersection_03_14 = _leg_line_03.getIntersection(_leg_line_14);
	SVector2 _intersection_03_25 = _leg_line_03.getIntersection(_leg_line_25);
	SVector2 _intersection_14_25 = _leg_line_14.getIntersection(_leg_line_25);

	//�O�p�`���쐬����D
	(*out_poly).reset();
	(*out_poly).addVertex(_intersection_03_14);
	(*out_poly).addVertex(_intersection_03_25);
	(*out_poly).addVertex(_intersection_14_25);

	if (_intersection_03_14.x > _intersection_03_25.x)
	{
		(*out_com_pattern) = ComType::EComPattern::center_back;
	}
	else
	{
		(*out_com_pattern) = ComType::EComPattern::center_front;
	}

	return;
}

bool ComCandidatePolygonMaker::checkPolygon(const my_vec::SPolygon2& _poly) const
{
	//���������̂� 3 or 4 or 5�p�`�̂�
	if (_poly.getVertexNum() == 3 || _poly.getVertexNum() == 4 || _poly.getVertexNum() == 5)
	{
		//�ʑ��p�`�ł��邩���m�F����
		if (_poly.isConvex() == true)
		{
			return true;
		}
	}

	return false;
}
