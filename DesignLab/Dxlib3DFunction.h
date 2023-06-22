#pragma once
#include "Dxlib.h"
#include "MyVector.h"


namespace myDxlib3DFunc 
{
	//! @brief 3D�������s����ŕK�v�ȏ������������܂Ƃ߂����́D
	void initDxlib3D();

	//! @brief Dxlib�̍��W������VECTOR�ƁC���̃v���O�����Ŏg�p���Ă���SVector��ϊ�����D<br>���{�b�g���W�n�͉E����W�n�CDxlib�͍�����W�n(�H�w�͉E��E�Q�[�����C�u�����͍��肪�����C������)�Ȃ̂�y�𔽓]����D
	//! @param [in] _vec �ϊ��O�̍��W�D
	//! @return VECTOR �ϊ���̍��W�D
	inline VECTOR convertToDxVec(const my_vec::SVector& _vec) { return VGet(_vec.x, -_vec.y, _vec.z); }

	//! @brief 3D��Ԃɗ����̂�`�悷��D
	//! @param [in] _center_pos �����̂̒��S�̍��W�D
	//! @param [in] _side_len �����̂�1�ӂ̒����D
	//! @param [in] _color �����̂̐F�Ddxlib��GetColor�Ŏ擾����.
	void drawCube3D(const VECTOR _center_pos, const float _side_len, const unsigned int _color);

	//! @brief 3D��Ԃɗ����̂�`�悷��D�����̂̏�ʂ̒��S�̍��W����`�悷��D
	//! @param [in] _center_pos �����̂̏�ʂ̒��S�̍��W�D
	//! @param [in] _side_len �����̂�1�ӂ̒����D
	//! @param [in] _color �����̂̐F�Ddxlib��GetColor�Ŏ擾����.
	void drawCube3DWithTopPos(const VECTOR _top_pos, const float _side_len, const unsigned int _color);

	//! @brief 3D��ԂɘZ�p�`��`�悷��D
	//! @param [in] _vertex �e���_�̍��W�D
	//! @param [in] _color �F�Ddxlib��GetColor�Ŏ擾����.
	void drawHexagon(const VECTOR _vertex[6], const unsigned int _color);

	//! @brief 3D��ԂɘZ�p����`�悷��D
	//! @param _vertex �Z�p���̐^�񒆂̊e���_�̍��W�C���ꕽ�ʏ�ɂ�����̂ɂ��邱�ƁD
	//! @param _height �Z�p���̍����D
	//! @param _color �F�Ddxlib��GetColor�Ŏ擾����.
	void drawHexagonalPrism(const VECTOR _vertex[6], const float _height, const unsigned int _color);
}


//! @file Dxlib3DFunction.h
//! @brief Dxlib��3D�\�����s�������������������֐����܂Ƃ߂����́D
//! @author ���J��

//! @namespace myDxlib3DFunc
//! @brief Dxlib��3D�\�����s���������������������́D
//! @details Dxlib �� 3D�ŕ\������@�\�̓n�b�L�������Ď��ʂقǎg���Â炢�̂ŁC�����ł�����x�g���₷���Ȃ�悤�ɏ��������D
//! @author ���J��
