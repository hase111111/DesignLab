#include "com_candidate_polygon_maker.h"

#include "designlab_line.h"
#include "hexapod_const.h"


void ComCandidatePolygonMaker::makeCandidatePolygon(const SNode& node, std::pair<dl_vec::SPolygon2, EDiscreteComPos> output_poly[MAKE_POLYGON_NUM]) const
{
	if (!mp_calculator) { return; }


	dl_vec::SVector2 leg_pos_xy[HexapodConst::LEG_NUM];	//XY���ʂɎˉe�����r�ʒu���Z�o����

	for (int i = 0; i < HexapodConst::LEG_NUM; i++)
	{
		leg_pos_xy[i] = mp_calculator->getGlobalLegPosition(i, node.leg_pos[i], node.global_center_of_mass, node.rot, false).projectedXY();		//�r�ʒu(�O���[�o�����W)��XY���ʂɎˉe����
	}

	//���S���͂ނ悤��4�p�`���쐬����
	makeCandidateBox(leg_pos_xy, 0, &output_poly[0].first); output_poly[0].second = EDiscreteComPos::FRONT_LEFT;
	makeCandidateBox(leg_pos_xy, 1, &output_poly[1].first); output_poly[1].second = EDiscreteComPos::BACK_LEFT;
	makeCandidateBox(leg_pos_xy, 2, &output_poly[2].first); output_poly[2].second = EDiscreteComPos::BACK;
	makeCandidateBox(leg_pos_xy, 3, &output_poly[3].first); output_poly[3].second = EDiscreteComPos::BACK_RIGHT;
	makeCandidateBox(leg_pos_xy, 4, &output_poly[4].first); output_poly[4].second = EDiscreteComPos::FRONT_RIGHT;
	makeCandidateBox(leg_pos_xy, 5, &output_poly[5].first); output_poly[5].second = EDiscreteComPos::FRONT;

	//���S��3�p�`���쐬����
	makeCandidateTriangle(leg_pos_xy, &output_poly[6].first, &output_poly[6].second);

	//�����������p�`�����������ǂ������`�F�b�N���C�ُ�Ȃ��͍̂폜����
	if (DO_CHECK_POLYGON)
	{
		for (int i = 0; i < MAKE_POLYGON_NUM; ++i)
		{
			if (!checkPolygon(output_poly[i].first))
			{
				output_poly[i].second = EDiscreteComPos::ERROR_POS;
			}
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

void ComCandidatePolygonMaker::makeCandidateBox(const dl_vec::SVector2 leg_pos[HexapodConst::LEG_NUM], const int start_leg_num, dl_vec::SPolygon2* output_poly) const
{
	//�r�ʒu����Ō��ԁD���̌�_����d�S���n�_�����݂��鑽�p�`�����߂�
	dl_vec::SLine2 leg_line_02(leg_pos[(start_leg_num + 0) % HexapodConst::LEG_NUM], leg_pos[(start_leg_num + 2) % HexapodConst::LEG_NUM]);
	dl_vec::SLine2 leg_line_03(leg_pos[(start_leg_num + 0) % HexapodConst::LEG_NUM], leg_pos[(start_leg_num + 3) % HexapodConst::LEG_NUM]);
	dl_vec::SLine2 leg_line_14(leg_pos[(start_leg_num + 1) % HexapodConst::LEG_NUM], leg_pos[(start_leg_num + 4) % HexapodConst::LEG_NUM]);
	dl_vec::SLine2 leg_line_15(leg_pos[(start_leg_num + 1) % HexapodConst::LEG_NUM], leg_pos[(start_leg_num + 5) % HexapodConst::LEG_NUM]);
	dl_vec::SLine2 leg_line_25(leg_pos[(start_leg_num + 2) % HexapodConst::LEG_NUM], leg_pos[(start_leg_num + 5) % HexapodConst::LEG_NUM]);

	//��_(intersection)�����߂�
	dl_vec::SVector2 intersection_02_14 = leg_line_02.getIntersection(leg_line_14);
	dl_vec::SVector2 intersection_02_15 = leg_line_02.getIntersection(leg_line_15);
	dl_vec::SVector2 intersection_03_14 = leg_line_03.getIntersection(leg_line_14);
	dl_vec::SVector2 intersection_03_15 = leg_line_03.getIntersection(leg_line_15);

	//���S��0�Ԃ̋r�ʒu�����񂾐��������߂�
	dl_vec::SLine2 leg_line_0_center(leg_pos[(start_leg_num + 0) % HexapodConst::LEG_NUM], intersection_03_14);

	//���p�`����

	(*output_poly).reset();

	if (leg_line_0_center.hasIntersection(leg_line_25))
	{
		//��_������ꍇ�C5�p�`�̑��p�`���쐬����
		dl_vec::SVector2 intersection_03_25 = leg_line_03.getIntersection(leg_line_25);
		dl_vec::SVector2 intersection_14_25 = leg_line_14.getIntersection(leg_line_25);

		(*output_poly).addVertexCheckForDuplicates(intersection_03_15);
		(*output_poly).addVertexCheckForDuplicates(intersection_02_15);
		(*output_poly).addVertexCheckForDuplicates(intersection_02_14);
		(*output_poly).addVertexCheckForDuplicates(intersection_14_25);
		(*output_poly).addVertexCheckForDuplicates(intersection_03_25);
	}
	else
	{
		//��_���Ȃ��ꍇ�C���ɋ��߂�4�_�ŁC4�p�`�̑��p�`���쐬����
		(*output_poly).addVertex(intersection_03_15);
		(*output_poly).addVertex(intersection_02_15);
		(*output_poly).addVertex(intersection_02_14);
		(*output_poly).addVertex(intersection_03_14);
	}
}

void ComCandidatePolygonMaker::makeCandidateTriangle(const dl_vec::SVector2 leg_pos[HexapodConst::LEG_NUM], dl_vec::SPolygon2* out_poly, EDiscreteComPos* out_com_pattern) const
{
	dl_vec::SLine2 leg_line_03(leg_pos[0], leg_pos[3]);
	dl_vec::SLine2 leg_line_14(leg_pos[1], leg_pos[4]);
	dl_vec::SLine2 leg_line_25(leg_pos[2], leg_pos[5]);

	//��_(intersection)�����߂�
	dl_vec::SVector2 intersection_03_14 = leg_line_03.getIntersection(leg_line_14);
	dl_vec::SVector2 intersection_03_25 = leg_line_03.getIntersection(leg_line_25);
	dl_vec::SVector2 intersection_14_25 = leg_line_14.getIntersection(leg_line_25);

	//�O�p�`���쐬����D
	(*out_poly).reset();
	(*out_poly).addVertex(intersection_03_14);
	(*out_poly).addVertex(intersection_03_25);
	(*out_poly).addVertex(intersection_14_25);

	if (intersection_03_14.x > intersection_03_25.x)
	{
		(*out_com_pattern) = EDiscreteComPos::CENTER_BACK;
	}
	else
	{
		(*out_com_pattern) = EDiscreteComPos::CENTER_FRONT;
	}

	return;
}

bool ComCandidatePolygonMaker::checkPolygon(const dl_vec::SPolygon2& _poly) const
{
	//���������̂� 3 or 4 or 5�p�`�̂�
	if (_poly.getVertexNum() == 3 || _poly.getVertexNum() == 4 || _poly.getVertexNum() == 5)
	{
		//�ʑ��p�`�ł��邩���m�F����
		if (_poly.isConvex())
		{
			return true;
		}
	}

	return false;
}
