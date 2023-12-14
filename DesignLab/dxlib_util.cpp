#include "dxlib_util.h"

#include <Dxlib.h>

#include "designlab_vector3.h"
#include "graphic_const.h"


namespace designlab
{
	namespace dxlib_util
	{

		void InitDxlib3DSetting(const bool high_quality)
		{
			if (high_quality)
			{
				SetUseLighting(TRUE);
				SetLightEnable(TRUE);

				// ���C�g�̐ݒ�
				ChangeLightTypeDir(ConvertToDxlibVec(Vector3::GetUpVec()));
			}
			else
			{
				SetUseLighting(FALSE);
				SetLightEnable(FALSE);
			}
		}


		void SetZBufferEnable()
		{
			// �y�o�b�t�@��L���ɂ���
			SetUseZBuffer3D(TRUE);

			// �y�o�b�t�@�ւ̏������݂�L���ɂ���
			SetWriteZBuffer3D(TRUE);
		}


		void DrawCube3D(const VECTOR& center_pos, const float side_len, const unsigned int color)
		{
			//�����̂�8�̒��_�����̂ŁC�����̍��W���v�Z����D
			const std::array<VECTOR, 8> vertex =
			{
				VGet(center_pos.x - side_len / 2,center_pos.y - side_len / 2,center_pos.z - side_len / 2),
				VGet(center_pos.x + side_len / 2,center_pos.y - side_len / 2,center_pos.z - side_len / 2),
				VGet(center_pos.x + side_len / 2,center_pos.y - side_len / 2,center_pos.z + side_len / 2),
				VGet(center_pos.x - side_len / 2,center_pos.y - side_len / 2,center_pos.z + side_len / 2),
				VGet(center_pos.x - side_len / 2,center_pos.y + side_len / 2,center_pos.z - side_len / 2),
				VGet(center_pos.x + side_len / 2,center_pos.y + side_len / 2,center_pos.z - side_len / 2),
				VGet(center_pos.x + side_len / 2,center_pos.y + side_len / 2,center_pos.z + side_len / 2),
				VGet(center_pos.x - side_len / 2,center_pos.y + side_len / 2,center_pos.z + side_len / 2)
			};

			// 3D�`��̊֐���3�p�`����{�P�ʂƂ���̂ŁC4�p�`�̖ʂ𒣂肽���ꍇ�́C2�̎O�p�`��g�ݍ��킹��K�v������D�܂�C6�ʁ~2��12�̎O�p�`�ŗ����̂��`��ł���D

			DrawTriangle3D(vertex[0], vertex[1], vertex[2], color, TRUE);
			DrawTriangle3D(vertex[2], vertex[3], vertex[0], color, TRUE);

			DrawTriangle3D(vertex[4], vertex[5], vertex[6], color, TRUE);
			DrawTriangle3D(vertex[6], vertex[7], vertex[4], color, TRUE);

			DrawTriangle3D(vertex[4], vertex[7], vertex[0], color, TRUE);
			DrawTriangle3D(vertex[0], vertex[7], vertex[3], color, TRUE);

			DrawTriangle3D(vertex[1], vertex[2], vertex[5], color, TRUE);
			DrawTriangle3D(vertex[5], vertex[6], vertex[2], color, TRUE);

			DrawTriangle3D(vertex[0], vertex[1], vertex[5], color, TRUE);
			DrawTriangle3D(vertex[5], vertex[4], vertex[0], color, TRUE);

			DrawTriangle3D(vertex[2], vertex[3], vertex[7], color, TRUE);
			DrawTriangle3D(vertex[7], vertex[6], vertex[2], color, TRUE);

		}


		void DrawCube3DWithTopPos(const VECTOR& top_pos, const float side_len, const unsigned int color)
		{
			DrawCube3D(VSub(top_pos, VGet(0, 0, side_len / 2)), side_len, color);
		}


		void DrawHexagon(const std::array<VECTOR, 6>& vertex, const unsigned int color)
		{
			// 3D�`��̊֐���3�p�`����{�P�ʂƂ���̂ŁC6�p�`�̖ʂ𒣂肽���ꍇ�́C4�̎O�p�`��g�ݍ��킹��K�v������D
			DrawTriangle3D(vertex[0], vertex[1], vertex[5], color, TRUE);
			DrawTriangle3D(vertex[1], vertex[2], vertex[4], color, TRUE);
			DrawTriangle3D(vertex[1], vertex[4], vertex[5], color, TRUE);
			DrawTriangle3D(vertex[2], vertex[3], vertex[4], color, TRUE);
		}


		void DrawHexagonalPrism(const std::array<VECTOR, 6>& vertex, const float height, const unsigned int color)
		{
			// 6�p�`�ʂ̖@�������̃x�N�g�����擾����D����Ă��鏈���Ƃ��ẮC���_0����1�֍s���x�N�g����v01�C���l�ɒ��_0����2�֍s���x�N�g����v02�Ƃ���ƁC
			// v01��v02�̊O��(Cross)���Ƃ�Ɩ@�������̃x�N�g�����擾�ł��邽�߁C�����P�ʃx�N�g���ɕϊ�(Norm�C�m�[�}���C�Y�̂���)���C�����̔��������{�ɂ����D
			const VECTOR center_to_top = VScale(VNorm(VCross(VSub(vertex[0], vertex[1]), VSub(vertex[0], vertex[2]))), height / 2.0f);

			//��ʂ̒��_�D
			const std::array<VECTOR, 6> vertex_top =
			{
				VAdd(vertex[0],center_to_top),
				VAdd(vertex[1],center_to_top),
				VAdd(vertex[2],center_to_top),
				VAdd(vertex[3],center_to_top),
				VAdd(vertex[4],center_to_top),
				VAdd(vertex[5],center_to_top)
			};

			//��ʂ̒��_
			const std::array<VECTOR, 6> vertex_bottom =
			{
				VSub(vertex[0],center_to_top),
				VSub(vertex[1],center_to_top),
				VSub(vertex[2],center_to_top),
				VSub(vertex[3],center_to_top),
				VSub(vertex[4],center_to_top),
				VSub(vertex[5],center_to_top)
			};

			DrawHexagon(vertex_top, color);		//��ʂ�`�悷��D
			DrawHexagon(vertex_bottom, color);	//��ʂ�`�悷��D

			//���ʂ�`�悵�Ă����D���ʂ͎l�p�`6�ō\�������̂ŁC3�p�`��12�K�v�ɂȂ�D
			for (int i = 0; i < 6; i++)
			{
				DrawTriangle3D(vertex_top[i % 6], vertex_top[(i + 1) % 6], vertex_bottom[i % 6], color, TRUE);
				DrawTriangle3D(vertex_top[(i + 1) % 6], vertex_bottom[i % 6], vertex_bottom[(i + 1) % 6], color, TRUE);
			}

		}

	}	//namespace dxlib_util

}	//namespace designlab