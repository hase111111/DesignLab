#pragma once
#include "Dxlib.h"
#include "vectorFunc.h"

// Dxlib �� 3D�ŕ\������@�\�̓n�b�L�������Ď��ʂقǎg���Â炢�̂ŁC�����ł�����x�g���₷���Ȃ�悤�ɏ��������D
namespace myDxlib3DFunc 
{
	// 3D�������s����ŕK�v�ȏ������������܂Ƃ߂����́D
	void initDxlib3D();

	// Dxlib�̍��W������VECTOR�ƁC���̃v���O�����Ŏg�p���Ă���SVector��ϊ�����D
	inline VECTOR convertToDxVec(const myvector::SVector& _vec) { return VGet(_vec.x, _vec.y, _vec.z); }

	// 3D��Ԃɗ����̂�`�悷��D_center_pos�c�����̂̒��S�̍��W�D_side_len�c�����̂�1�ӂ̒����D_color�c�����̂̐F�Ddxlib��GetColor�Ŏ擾����.
	void drawCube3D(const VECTOR _center_pos, const float _side_len, const unsigned int _color);

	// 3D��Ԃɗ����̂�`�悷��D_center_pos�c�����̂̏�ʂ̒��S�̍��W�D_side_len�c�����̂�1�ӂ̒����D_color�c�����̂̐F�Ddxlib��GetColor�Ŏ擾����.
	void drawCube3DWithTopPos(const VECTOR _top_pos, const float _side_len, const unsigned int _color);

	// 3D��ԂɘZ�p�`��`�悷��D_vertex�c�e���_�̍��W�D_color�c�F�Ddxlib��GetColor�Ŏ擾����.
	void drawHexagon(const VECTOR _vertex[6], const unsigned int _color);

	// 3D��ԂɘZ�p����`�悷��D
	//_vertex�c�Z�p���̐^�񒆂̊e���_�̍��W�C���ꕽ�ʏ�ɂ�����̂ɂ��邱�ƁD_height�c�Z�p���̍����D_color�c�F�Ddxlib��GetColor�Ŏ擾����.
	void drawHexagonalPrism(const VECTOR _vertex[6], const float _height, const unsigned int _color);
}
