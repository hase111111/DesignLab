#pragma once

#include "Dxlib.h"

#include "designlab_vector.h"
#include "hexapod_const.h"


//! @namespace dl_dxlib
//! @date 2023/08/14
//! @author ���J��
//! @brief Dxlib��3D�\�����s���������������������́D
//! @details Dxlib �� 3D�ŕ\������@�\�̓n�b�L�������Ď��ʂقǎg���Â炢�̂ŁC�����ł�����x�g���₷���Ȃ�悤�ɏ��������D
namespace dl_dxlib
{
	//! @brief 3D�������s����ŕK�v�ȏ������������܂Ƃ߂����́D
	void initDxlib3D();


	//! @brief Dxlib�̍��W������VECTOR�ƁC���̃v���O�����Ŏg�p���Ă���SVector��ϊ�����D<br>���{�b�g���W�n�͉E����W�n�CDxlib�͍�����W�n(�H�w�͉E��E�Q�[�����C�u�����͍��肪�����C������)�Ȃ̂�y�𔽓]����D
	//! @param [in] vec �ϊ��O�̍��W�D
	//! @return VECTOR �ϊ���̍��W�D
	inline VECTOR convertToDxVec(const dl_vec::SVector& vec) { return VGet(vec.x, -vec.y, vec.z); }


	//! @brief �f�t�H���g���ƕ`�揈�������������ɕ`�悳��邪�C�����Z�o�b�t�@���g�p���ĉ��s�����l�����ĕ`�悷��悤�ɂ���D
	void setZBufferEnable();


	//! @brief 3D��Ԃɗ����̂�`�悷��D
	//! @param [in] center_pos �����̂̒��S�̍��W�D
	//! @param [in] side_len �����̂�1�ӂ̒����D
	//! @param [in] color �����̂̐F�Ddxlib��GetColor�Ŏ擾����.
	void drawCube3D(const VECTOR& center_pos, const float side_len, const unsigned int color);


	//! @brief 3D��Ԃɗ����̂�`�悷��D�����̂̏�ʂ̒��S�̍��W����`�悷��D
	//! @param [in] center_pos �����̂̏�ʂ̒��S�̍��W�D
	//! @param [in] side_len �����̂�1�ӂ̒����D
	//! @param [in] color �����̂̐F�Ddxlib��GetColor�Ŏ擾����.
	void drawCube3DWithTopPos(const VECTOR& top_pos, const float side_len, const unsigned int color);


	//! @brief 3D��ԂɘZ�p�`��`�悷��D
	//! @param [in] vertex �e���_�̍��W�D
	//! @param [in] color �F�Ddxlib��GetColor�Ŏ擾����.
	void drawHexagon(const VECTOR vertex[HexapodConst::LEG_NUM], const unsigned int color);


	//! @brief 3D��ԂɘZ�p����`�悷��D
	//! @param vertex �Z�p���̐^�񒆂̊e���_�̍��W�C���ꕽ�ʏ�ɂ�����̂ɂ��邱�ƁD
	//! @param height �Z�p���̍����D
	//! @param color �F�Ddxlib��GetColor�Ŏ擾����.
	void drawHexagonalPrism(const VECTOR vertex[HexapodConst::LEG_NUM], const float height, const unsigned int color);

} // namespace dl_dxlib


//! @file designlab_dxlib.h
//! @date 2023/08/14
//! @author ���J��
//! @brief Dxlib��3D�\�����s�������������������֐����܂Ƃ߂����́D
//! @n �s�� : @lineinfo
