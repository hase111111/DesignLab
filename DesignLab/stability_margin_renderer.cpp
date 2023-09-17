#include "stability_margin_renderer.h"

#include <vector>

#include "DxLib.h"

#include "leg_state.h"
#include "designlab_dxlib.h"
#include "designlab_polygon.h"


StabilityMarginRenderer::StabilityMarginRenderer() : kMarginColor(GetColor(0, 255, 0)), kMarginErrorColor(GetColor(255, 0, 0)), kAlpha(128)
{
}


void StabilityMarginRenderer::Draw(const SNode& node) const
{
	dl_vec::SPolygon2 polygon_xy;			//���ʂɓ��e�������p�`

	std::vector<dl_vec::SVector> polygon;	//���p�`�̒��_

	dl_vec::SVector polygon_sum{0, 0, 0};	//���p�`�̒��_�̍��v


	for (int i = 0; i < HexapodConst::LEG_NUM; i++)
	{
		if (dl_leg::isGrounded(node.leg_state, i))
		{
			polygon.push_back(m_hexapod_state_calclator.getGlobalLegPos(node, i, true));

			polygon.back() += dl_vec::SVector{0, 0, 5};

			polygon_xy.addVertex(polygon.back().projectedXY());

			polygon_sum += polygon.back();
		}

	}

	dl_vec::SVector center = polygon_sum / static_cast<float>(polygon.size());


	for (size_t i = 0; i < polygon.size(); i++)
	{
		VECTOR poly[3] = {
			dl_dxlib::convertToDxVec(polygon[i]),
			dl_dxlib::convertToDxVec(polygon[(i + 1) % polygon.size()]),
			dl_dxlib::convertToDxVec(center)
		};

		SetDrawBlendMode(DX_BLENDMODE_ALPHA, kAlpha);

		if (polygon_xy.isInside(node.global_center_of_mass.projectedXY()))
		{
			DrawTriangle3D(poly[0], poly[1], poly[2], kMarginColor, TRUE);
		}
		else
		{
			DrawTriangle3D(poly[0], poly[1], poly[2], kMarginErrorColor, TRUE);
		}


		SetDrawBlendMode(DX_BLENDMODE_NOBLEND, 0);
	}

	//���˂����d�S��`�悷��
	VECTOR projected_center = dl_dxlib::convertToDxVec({ node.global_center_of_mass.x,node.global_center_of_mass.y, center.z + 10 });

	DrawSphere3D(projected_center, 5, 10, 10, GetColor(255, 255, 255), TRUE);
}
