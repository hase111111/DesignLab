#include "Dxlib3DFunction.h"

#include "DxLib.h"


#include "graphic_const.h"


void myDxlib3DFunc::initDxlib3D()
{
	//�J�����̕`��͈͂�ݒ肷��
	SetCameraNearFar(GraphicConst::CAMERA_NEAR, GraphicConst::CAMERA_FAR);

	SetUseLighting(FALSE);					// ���C�e�B���O�̌v�Z�����Ȃ��悤�ɐݒ��ύX	
	SetUseBackCulling(FALSE);				// �|���S���̗��ʂ�`�悷��D
	SetUseZBuffer3D(TRUE);					// �y�o�b�t�@��L���ɂ���
	SetWriteZBuffer3D(TRUE);				// �y�o�b�t�@�ւ̏������݂�L���ɂ���
	SetFogEnable(FALSE);					// �t�H�O�͎g�p���Ȃ��D
}


void myDxlib3DFunc::drawCube3D(const VECTOR _center_pos, const float _side_len, const unsigned int _color)
{
	//�����̂�8�̒��_�����̂ŁC�����̍��W���v�Z����D
	const VECTOR _vertex[8] =
	{
		VGet(_center_pos.x - _side_len / 2,_center_pos.y - _side_len / 2,_center_pos.z - _side_len / 2),
		VGet(_center_pos.x + _side_len / 2,_center_pos.y - _side_len / 2,_center_pos.z - _side_len / 2),
		VGet(_center_pos.x + _side_len / 2,_center_pos.y - _side_len / 2,_center_pos.z + _side_len / 2),
		VGet(_center_pos.x - _side_len / 2,_center_pos.y - _side_len / 2,_center_pos.z + _side_len / 2),
		VGet(_center_pos.x - _side_len / 2,_center_pos.y + _side_len / 2,_center_pos.z - _side_len / 2),
		VGet(_center_pos.x + _side_len / 2,_center_pos.y + _side_len / 2,_center_pos.z - _side_len / 2),
		VGet(_center_pos.x + _side_len / 2,_center_pos.y + _side_len / 2,_center_pos.z + _side_len / 2),
		VGet(_center_pos.x - _side_len / 2,_center_pos.y + _side_len / 2,_center_pos.z + _side_len / 2)
	};

	// 3D�`��̊֐���3�p�`����{�P�ʂƂ���̂ŁC4�p�`�̖ʂ𒣂肽���ꍇ�́C2�̎O�p�`��g�ݍ��킹��K�v������D�܂�C6�ʁ~2��12�̎O�p�`�ŗ����̂��`��ł���D

	DrawTriangle3D(_vertex[0], _vertex[1], _vertex[2], _color, TRUE);
	DrawTriangle3D(_vertex[2], _vertex[3], _vertex[0], _color, TRUE);

	DrawTriangle3D(_vertex[4], _vertex[5], _vertex[6], _color, TRUE);
	DrawTriangle3D(_vertex[6], _vertex[7], _vertex[4], _color, TRUE);

	DrawTriangle3D(_vertex[4], _vertex[7], _vertex[0], _color, TRUE);
	DrawTriangle3D(_vertex[0], _vertex[7], _vertex[3], _color, TRUE);

	DrawTriangle3D(_vertex[1], _vertex[2], _vertex[5], _color, TRUE);
	DrawTriangle3D(_vertex[5], _vertex[6], _vertex[2], _color, TRUE);

	DrawTriangle3D(_vertex[0], _vertex[1], _vertex[5], _color, TRUE);
	DrawTriangle3D(_vertex[5], _vertex[4], _vertex[0], _color, TRUE);

	DrawTriangle3D(_vertex[2], _vertex[3], _vertex[7], _color, TRUE);
	DrawTriangle3D(_vertex[7], _vertex[6], _vertex[2], _color, TRUE);

}


void myDxlib3DFunc::drawCube3DWithTopPos(const VECTOR _top_pos, const float _side_len, const unsigned int _color)
{
	drawCube3D(VSub(_top_pos, VGet(0, 0, _side_len / 2)), _side_len, _color);
}


void myDxlib3DFunc::drawHexagon(const VECTOR _vertex[6], const unsigned int _color)
{
	// 3D�`��̊֐���3�p�`����{�P�ʂƂ���̂ŁC6�p�`�̖ʂ𒣂肽���ꍇ�́C4�̎O�p�`��g�ݍ��킹��K�v������D
	DrawTriangle3D(_vertex[0], _vertex[1], _vertex[5], _color, TRUE);
	DrawTriangle3D(_vertex[1], _vertex[2], _vertex[4], _color, TRUE);
	DrawTriangle3D(_vertex[1], _vertex[4], _vertex[5], _color, TRUE);
	DrawTriangle3D(_vertex[2], _vertex[3], _vertex[4], _color, TRUE);
}


void myDxlib3DFunc::drawHexagonalPrism(const VECTOR _vertex[6], const float _height, const unsigned int _color)
{
	//6�p�`�ʂ̖@�������̃x�N�g�����擾����D����Ă��鏈���Ƃ��ẮC���_0����1�֍s���x�N�g����v01�C���l�ɒ��_0����2�֍s���x�N�g����v02�Ƃ���ƁC
	// v01��v02�̊O��(Cross)���Ƃ�Ɩ@�������̃x�N�g�����擾�ł��邽�߁C�����P�ʃx�N�g���ɕϊ�(Norm�C�m�[�}���C�Y�̂���)���C�����̔��������{�ɂ����D
	const VECTOR _center_to_top = VScale(VNorm(VCross(VSub(_vertex[0], _vertex[1]), VSub(_vertex[0], _vertex[2]))), _height / 2.0f);

	//��ʂ̒��_�D
	const VECTOR _vertex_top[6] =
	{
		VAdd(_vertex[0],_center_to_top),
		VAdd(_vertex[1],_center_to_top),
		VAdd(_vertex[2],_center_to_top),
		VAdd(_vertex[3],_center_to_top),
		VAdd(_vertex[4],_center_to_top),
		VAdd(_vertex[5],_center_to_top)
	};

	//��ʂ̒��_
	const VECTOR _vertex_bottom[6] =
	{
		VSub(_vertex[0],_center_to_top),
		VSub(_vertex[1],_center_to_top),
		VSub(_vertex[2],_center_to_top),
		VSub(_vertex[3],_center_to_top),
		VSub(_vertex[4],_center_to_top),
		VSub(_vertex[5],_center_to_top)
	};

	drawHexagon(_vertex_top, _color);		//��ʂ�`�悷��D
	drawHexagon(_vertex_bottom, _color);	//��ʂ�`�悷��D

	//���ʂ�`�悵�Ă����D���ʂ͎l�p�`6�ō\�������̂ŁC3�p�`��12���K�v�ɂȂ�D
	for (int i = 0; i < 6; i++)
	{
		DrawTriangle3D(_vertex_top[i % 6], _vertex_top[(i + 1) % 6], _vertex_bottom[i % 6], _color, TRUE);
		DrawTriangle3D(_vertex_top[(i + 1) % 6], _vertex_bottom[i % 6], _vertex_bottom[(i + 1) % 6], _color, TRUE);
	}

}
