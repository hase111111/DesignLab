#include "designlab_dxlib.h"

#include "DxLib.h"

#include "graphic_const.h"


void dl_dxlib::initDxlib3D()
{
	//�J�����̕`��͈͂�ݒ肷��
	SetCameraNearFar(GraphicConst::CAMERA_NEAR, GraphicConst::CAMERA_FAR);

	SetUseLighting(FALSE);					// ���C�e�B���O�̌v�Z�����Ȃ��悤�ɐݒ��ύX	
	SetUseBackCulling(FALSE);				// �|���S���̗��ʂ�`�悷��D
	SetUseZBuffer3D(TRUE);					// �y�o�b�t�@��L���ɂ���
	SetWriteZBuffer3D(TRUE);				// �y�o�b�t�@�ւ̏������݂�L���ɂ���
	SetFogEnable(FALSE);					// �t�H�O�͎g�p���Ȃ��D
}


void dl_dxlib::drawCube3D(const VECTOR& center_pos, const float side_len, const unsigned int color)
{
	//�����̂�8�̒��_�����̂ŁC�����̍��W���v�Z����D
	const VECTOR _vertex[8] =
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

	DrawTriangle3D(_vertex[0], _vertex[1], _vertex[2], color, TRUE);
	DrawTriangle3D(_vertex[2], _vertex[3], _vertex[0], color, TRUE);

	DrawTriangle3D(_vertex[4], _vertex[5], _vertex[6], color, TRUE);
	DrawTriangle3D(_vertex[6], _vertex[7], _vertex[4], color, TRUE);

	DrawTriangle3D(_vertex[4], _vertex[7], _vertex[0], color, TRUE);
	DrawTriangle3D(_vertex[0], _vertex[7], _vertex[3], color, TRUE);

	DrawTriangle3D(_vertex[1], _vertex[2], _vertex[5], color, TRUE);
	DrawTriangle3D(_vertex[5], _vertex[6], _vertex[2], color, TRUE);

	DrawTriangle3D(_vertex[0], _vertex[1], _vertex[5], color, TRUE);
	DrawTriangle3D(_vertex[5], _vertex[4], _vertex[0], color, TRUE);

	DrawTriangle3D(_vertex[2], _vertex[3], _vertex[7], color, TRUE);
	DrawTriangle3D(_vertex[7], _vertex[6], _vertex[2], color, TRUE);

}


void dl_dxlib::drawCube3DWithTopPos(const VECTOR& top_pos, const float side_len, const unsigned int color)
{
	drawCube3D(VSub(top_pos, VGet(0, 0, side_len / 2)), side_len, color);
}


void dl_dxlib::drawHexagon(const VECTOR vertex[HexapodConst::LEG_NUM], const unsigned int color)
{
	// 3D�`��̊֐���3�p�`����{�P�ʂƂ���̂ŁC6�p�`�̖ʂ𒣂肽���ꍇ�́C4�̎O�p�`��g�ݍ��킹��K�v������D
	DrawTriangle3D(vertex[0], vertex[1], vertex[5], color, TRUE);
	DrawTriangle3D(vertex[1], vertex[2], vertex[4], color, TRUE);
	DrawTriangle3D(vertex[1], vertex[4], vertex[5], color, TRUE);
	DrawTriangle3D(vertex[2], vertex[3], vertex[4], color, TRUE);
}


void dl_dxlib::drawHexagonalPrism(const VECTOR vertex[HexapodConst::LEG_NUM], const float height, const unsigned int color)
{
	//6�p�`�ʂ̖@�������̃x�N�g�����擾����D����Ă��鏈���Ƃ��ẮC���_0����1�֍s���x�N�g����v01�C���l�ɒ��_0����2�֍s���x�N�g����v02�Ƃ���ƁC
	// v01��v02�̊O��(Cross)���Ƃ�Ɩ@�������̃x�N�g�����擾�ł��邽�߁C�����P�ʃx�N�g���ɕϊ�(Norm�C�m�[�}���C�Y�̂���)���C�����̔��������{�ɂ����D
	const VECTOR kCenterToTop = VScale(VNorm(VCross(VSub(vertex[0], vertex[1]), VSub(vertex[0], vertex[2]))), height / 2.0f);

	//��ʂ̒��_�D
	const VECTOR kVertexTop[HexapodConst::LEG_NUM] =
	{
		VAdd(vertex[0],kCenterToTop),
		VAdd(vertex[1],kCenterToTop),
		VAdd(vertex[2],kCenterToTop),
		VAdd(vertex[3],kCenterToTop),
		VAdd(vertex[4],kCenterToTop),
		VAdd(vertex[5],kCenterToTop)
	};

	//��ʂ̒��_
	const VECTOR kVertexBottom[HexapodConst::LEG_NUM] =
	{
		VSub(vertex[0],kCenterToTop),
		VSub(vertex[1],kCenterToTop),
		VSub(vertex[2],kCenterToTop),
		VSub(vertex[3],kCenterToTop),
		VSub(vertex[4],kCenterToTop),
		VSub(vertex[5],kCenterToTop)
	};

	drawHexagon(kVertexTop, color);		//��ʂ�`�悷��D
	drawHexagon(kVertexBottom, color);	//��ʂ�`�悷��D

	//���ʂ�`�悵�Ă����D���ʂ͎l�p�`6�ō\�������̂ŁC3�p�`��12���K�v�ɂȂ�D
	for (int i = 0; i < HexapodConst::LEG_NUM; i++)
	{
		DrawTriangle3D(kVertexTop[i % HexapodConst::LEG_NUM], kVertexTop[(i + 1) % HexapodConst::LEG_NUM], kVertexBottom[i % HexapodConst::LEG_NUM], color, TRUE);
		DrawTriangle3D(kVertexTop[(i + 1) % HexapodConst::LEG_NUM], kVertexBottom[i % HexapodConst::LEG_NUM], kVertexBottom[(i + 1) % HexapodConst::LEG_NUM], color, TRUE);
	}

}
