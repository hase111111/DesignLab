#include "com_candidate_polygon_maker.h"

#include "cassert_define.h"
#include "designlab_line_segment2.h"
#include "hexapod_const.h"


ComCandidatePolygonMaker::ComCandidatePolygonMaker(const std::shared_ptr<const AbstractHexapodStateCalculator>& calc) : 
	calculator_ptr_(calc)
{
}

void ComCandidatePolygonMaker::MakeCandidatePolygon(const RobotStateNode& node, std::array<ComPosAndPolygon, MAKE_POLYGON_NUM>* output_poly) const
{
	assert(calculator_ptr_ != nullptr);	//nullptr�łȂ����Ƃ��m�F����
	assert(output_poly != nullptr);		//nullptr�łȂ����Ƃ��m�F����


	std::array<designlab::Vector2, HexapodConst::kLegNum> leg_pos_xy;	//XY���ʂɎˉe�����r�ʒu���Z�o����

	for (int i = 0; i < HexapodConst::kLegNum; i++)
	{
		leg_pos_xy[i] = calculator_ptr_->ConvertLegToGlobalCoordinate(node.leg_pos[i], i, node.global_center_of_mass, node.rot, false).ProjectedXY();		//�r�ʒu(�O���[�o�����W)��XY���ʂɎˉe����
	}

	//���S���͂ނ悤��4�p�`���쐬����
	MakeCandidateBox(leg_pos_xy, 0, &(*output_poly)[0].polygon); 
	(*output_poly)[0].com_pos = DiscreteComPos::kFrontLeft;
	(*output_poly)[0].is_able = true;

	MakeCandidateBox(leg_pos_xy, 1, &(*output_poly)[1].polygon); 
	(*output_poly)[1].com_pos = DiscreteComPos::kBackLeft;
	(*output_poly)[1].is_able = true;

	MakeCandidateBox(leg_pos_xy, 2, &(*output_poly)[2].polygon); 
	(*output_poly)[2].com_pos = DiscreteComPos::kBack;
	(*output_poly)[2].is_able = true;

	MakeCandidateBox(leg_pos_xy, 3, &(*output_poly)[3].polygon);
	(*output_poly)[3].com_pos = DiscreteComPos::kBackRight;
	(*output_poly)[3].is_able = true;

	MakeCandidateBox(leg_pos_xy, 4, &(*output_poly)[4].polygon); 
	(*output_poly)[4].com_pos = DiscreteComPos::kFrontRight;
	(*output_poly)[4].is_able = true;

	MakeCandidateBox(leg_pos_xy, 5, &(*output_poly)[5].polygon); 
	(*output_poly)[5].com_pos = DiscreteComPos::kFront;
	(*output_poly)[5].is_able = true;

	//���S��3�p�`���쐬����
	MakeCandidateTriangle(leg_pos_xy, &(*output_poly)[6]);

	//�����������p�`�����������ǂ������`�F�b�N���C�ُ�Ȃ��͍̂폜����
	if (kDoCheckPolygon)
	{
		for (int i = 0; i < MAKE_POLYGON_NUM; ++i)
		{
			if (!IsAblePolygon((*output_poly)[i].polygon))
			{
				(*output_poly)[i].is_able = false;
			}
		}
	}

}

void ComCandidatePolygonMaker::MakeCandidateBox(const std::array<designlab::Vector2, HexapodConst::kLegNum>& leg_pos, const int start_leg_num, designlab::Polygon2* output_poly) const
{
	//�r�ʒu����Ō��ԁD���̌�_����d�S���n�_�����݂��鑽�p�`�����߂�
	designlab::LineSegment2 leg_line_02(leg_pos[(start_leg_num + 0) % HexapodConst::kLegNum], leg_pos[(start_leg_num + 2) % HexapodConst::kLegNum]);
	designlab::LineSegment2 leg_line_03(leg_pos[(start_leg_num + 0) % HexapodConst::kLegNum], leg_pos[(start_leg_num + 3) % HexapodConst::kLegNum]);
	designlab::LineSegment2 leg_line_14(leg_pos[(start_leg_num + 1) % HexapodConst::kLegNum], leg_pos[(start_leg_num + 4) % HexapodConst::kLegNum]);
	designlab::LineSegment2 leg_line_15(leg_pos[(start_leg_num + 1) % HexapodConst::kLegNum], leg_pos[(start_leg_num + 5) % HexapodConst::kLegNum]);
	designlab::LineSegment2 leg_line_25(leg_pos[(start_leg_num + 2) % HexapodConst::kLegNum], leg_pos[(start_leg_num + 5) % HexapodConst::kLegNum]);

	//��_(intersection)�����߂�
	designlab::Vector2 intersection_02_14 = leg_line_02.GetIntersection(leg_line_14);
	designlab::Vector2 intersection_02_15 = leg_line_02.GetIntersection(leg_line_15);
	designlab::Vector2 intersection_03_14 = leg_line_03.GetIntersection(leg_line_14);
	designlab::Vector2 intersection_03_15 = leg_line_03.GetIntersection(leg_line_15);

	//���S��0�Ԃ̋r�ʒu�����񂾐��������߂�
	designlab::LineSegment2 leg_line_0_center(leg_pos[(start_leg_num + 0) % HexapodConst::kLegNum], intersection_03_14);

	//���p�`����

	(*output_poly).Reset();

	if (leg_line_0_center.HasIntersection(leg_line_25))
	{
		//��_������ꍇ�C5�p�`�̑��p�`���쐬����
		designlab::Vector2 intersection_03_25 = leg_line_03.GetIntersection(leg_line_25);
		designlab::Vector2 intersection_14_25 = leg_line_14.GetIntersection(leg_line_25);

		(*output_poly).AddVertexCheckForDuplicates(intersection_03_15);
		(*output_poly).AddVertexCheckForDuplicates(intersection_02_15);
		(*output_poly).AddVertexCheckForDuplicates(intersection_02_14);
		(*output_poly).AddVertexCheckForDuplicates(intersection_14_25);
		(*output_poly).AddVertexCheckForDuplicates(intersection_03_25);
	}
	else
	{
		//��_���Ȃ��ꍇ�C���ɋ��߂�4�_�ŁC4�p�`�̑��p�`���쐬����
		(*output_poly).AddVertex(intersection_03_15);
		(*output_poly).AddVertex(intersection_02_15);
		(*output_poly).AddVertex(intersection_02_14);
		(*output_poly).AddVertex(intersection_03_14);
	}
}

void ComCandidatePolygonMaker::MakeCandidateTriangle(const std::array<designlab::Vector2, HexapodConst::kLegNum>& leg_pos, ComPosAndPolygon* output) const
{
	designlab::LineSegment2 leg_line_03(leg_pos[0], leg_pos[3]);
	designlab::LineSegment2 leg_line_14(leg_pos[1], leg_pos[4]);
	designlab::LineSegment2 leg_line_25(leg_pos[2], leg_pos[5]);

	//��_(intersection)�����߂�
	designlab::Vector2 intersection_03_14 = leg_line_03.GetIntersection(leg_line_14);
	designlab::Vector2 intersection_03_25 = leg_line_03.GetIntersection(leg_line_25);
	designlab::Vector2 intersection_14_25 = leg_line_14.GetIntersection(leg_line_25);

	//�O�p�`���쐬����D
	output->polygon.Reset();
	output->polygon.AddVertex(intersection_03_14);
	output->polygon.AddVertex(intersection_03_25);
	output->polygon.AddVertex(intersection_14_25);

	if (intersection_03_14.x > intersection_03_25.x)
	{
		output->com_pos = DiscreteComPos::kCenterBack;
	}
	else
	{
		output->com_pos = DiscreteComPos::kCenterFront;
	}

	output->is_able = true;

	return;
}

bool ComCandidatePolygonMaker::IsAblePolygon(const designlab::Polygon2& _poly) const
{
	//���������̂� 3 or 4 or 5�p�`�̂�
	if (_poly.GetVertexNum() == 3 || _poly.GetVertexNum() == 4 || _poly.GetVertexNum() == 5)
	{
		//�ʑ��p�`�ł��邩���m�F����
		if (_poly.IsConvex())
		{
			return true;
		}
	}

	return false;
}