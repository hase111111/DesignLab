#include "ComCandidatePolygonMaker.h"
#include "MyLine.h"
#include "HexapodConst.h"

void ComCandidatePolygonMaker::makeCandidatePolygon(const SNode& _node, std::vector<my_vec::SPolygon2>& _output_poly)
{
	using namespace my_vec;

	_output_poly.clear();	// output���N���A


	for (int i = 0; i < HexapodConst::LEG_NUM; i++)
	{
		SVector2 _leg_pos_xy[HexapodConst::LEG_NUM];	//XY���ʂɎˉe�����r�ʒu

		for (int j = 0; j < HexapodConst::LEG_NUM; j++)
		{
			_leg_pos_xy[j] = _node.leg_pos[(i + 0) % HexapodConst::LEG_NUM].projectedXY();	//�r�ʒu��XY���ʂɎˉe����
		}

		//�r�ʒu����Ō��ԁD���̌�_����d�S���n�_�����݂��鑽�p�`�����߂�
		SLine2 _leg_line_02(_leg_pos_xy[0], _leg_pos_xy[2]);
		SLine2 _leg_line_03(_leg_pos_xy[0], _leg_pos_xy[3]);
		SLine2 _leg_line_14(_leg_pos_xy[1], _leg_pos_xy[4]);
		SLine2 _leg_line_15(_leg_pos_xy[1], _leg_pos_xy[5]);
		SLine2 _leg_line_25(_leg_pos_xy[2], _leg_pos_xy[5]);

		//��_�����߂�
		SVector2 _intersection_02_14 = _leg_line_02.getIntersection(_leg_line_14);
		SVector2 _intersection_02_15 = _leg_line_02.getIntersection(_leg_line_15);
		SVector2 _intersection_03_14 = _leg_line_03.getIntersection(_leg_line_14);
		SVector2 _intersection_03_15 = _leg_line_03.getIntersection(_leg_line_15);

		//���S��0�Ԃ̋r�ʒu�����񂾐��������߂�
		SLine2 _leg_line_0_center(_leg_pos_xy[0], _intersection_03_14);

		if (_leg_line_0_center.hasIntersection(_leg_line_25) == true)
		{
			//��_������ꍇ�C5�p�`�̑��p�`���쐬����
			SVector2 _intersection_03_25 = _leg_line_03.getIntersection(_leg_line_25);
			SVector2 _intersection_14_25 = _leg_line_14.getIntersection(_leg_line_25);

			SPolygon2 _poly;
			_poly.addVertex(_intersection_03_15);
			_poly.addVertex(_intersection_02_15);
			_poly.addVertex(_intersection_02_14);
			_poly.addVertex(_intersection_14_25);
			_poly.addVertex(_intersection_03_25);
			_output_poly.push_back(_poly);
		}
		else
		{
			//��_���Ȃ��ꍇ�C���ɋ��߂�4�_�ŁC4�p�`�̑��p�`���쐬����
			SPolygon2 _poly;
			_poly.addVertex(_intersection_03_15);
			_poly.addVertex(_intersection_02_15);
			_poly.addVertex(_intersection_02_14);
			_poly.addVertex(_intersection_03_14);
			_output_poly.push_back(_poly);
		}

	}

}
