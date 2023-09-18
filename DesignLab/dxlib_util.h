//! @file dxlib_util.h
//! @brief Dxlib��3D�\�����s�������������������֐����܂Ƃ߂����́D


#ifndef DESIGNLAB_DXLIB_H_
#define DESIGNLAB_DXLIB_H_

#include "Dxlib.h"

#include <array>

#include "designlab_vector.h"


namespace designlab
{

	//! @namespace dxlib_util
	//! @brief Dxlib��3D�\�����s���������������������́D
	//! @details Dxlib �� 3D�ŕ\������@�\�̓n�b�L�������Ď��ʂقǎg���Â炢�̂ŁC�����ł�����x�g���₷���Ȃ�悤�ɏ����������Ă܂Ƃ߂Ă����D

	namespace dxlib_util
	{
		//! @brief 3D�������s����ŕK�v�ȏ������������܂Ƃ߂����́D
		void InitDxlib3DSetting();


		//! @brief Dxlib�̍��W������VECTOR�ƁC���̃v���O�����Ŏg�p���Ă���Vector��ϊ�����D
		//! @n ���{�b�g���W�n�͉E����W�n�CDxlib�͍�����W�n(�H�w�͉E��E�Q�[�����C�u�����͍��肪�����C������)�Ȃ̂�y�𔽓]����D
		//! @param [in] vec �ϊ��O�̍��W�D
		//! @return VECTOR �ϊ���̍��W�D
		inline VECTOR ConvertToDxlibVec(const dl_vec::SVector& vec) { return VGet(vec.x, -vec.y, vec.z); }


		//! @brief ���̃v���O�����Ŏg�p���Ă���Vector�ƁCDxlib�̍��W������VECTOR��ϊ�����D
		//! @n ���{�b�g���W�n�͉E����W�n�CDxlib�͍�����W�n(�H�w�͉E��E�Q�[�����C�u�����͍��肪�����C������)�Ȃ̂�y�𔽓]����D
		//! @param [in] vec �ϊ��O�̍��W�D
		//! @return SVector �ϊ���̍��W�D
		inline dl_vec::SVector ConvertDesignLabVec(const VECTOR& vec) { return dl_vec::SVector(vec.x, -vec.y, vec.z); }


		//! @brief �f�t�H���g���ƕ`�揈�������������ɕ`�悳��邪�C�����Z�o�b�t�@���g�p���ĉ��s�����l�����ĕ`�悷��悤�ɂ���D
		//! @n �Ȃ񂩖��t���[�����s���Ȃ��Ⴂ���Ȃ����ۂ��H�d�l���悭�킩���
		void SetZBufferEnable();


		//! @brief 3D��Ԃɗ����̂�`�悷��D
		//! @param [in] center_pos �����̂̒��S�̍��W�D
		//! @param [in] side_len �����̂�1�ӂ̒����D
		//! @param [in] color �����̂̐F�Ddxlib��GetColor�Ŏ擾����.
		void DrawCube3D(const VECTOR& center_pos, float side_len, unsigned int color);


		//! @brief 3D��Ԃɗ����̂�`�悷��D�����̂̏�ʂ̒��S�̍��W����`�悷��D
		//! @param [in] center_pos �����̂̏�ʂ̒��S�̍��W�D
		//! @param [in] side_len �����̂�1�ӂ̒����D
		//! @param [in] color �����̂̐F�Ddxlib��GetColor�Ŏ擾����.
		void DrawCube3DWithTopPos(const VECTOR& top_pos, float side_len, unsigned int color);


		//! @brief 3D��ԂɘZ�p�`��`�悷��D�e�_�͓��ꕽ�ʏ�ɂ�����̂ɂ��Ȃ��ƁC�`�悪�c�ށD
		//! @param [in] vertex �e���_�̍��W�D
		//! @param [in] color �F�Ddxlib��GetColor�Ŏ擾����.
		void DrawHexagon(const std::array<VECTOR, 6>& vertex, unsigned int color);


		//! @brief 3D��ԂɘZ�p����`�悷��D
		//! @param vertex �Z�p���̐^�񒆂̊e���_�̍��W�C���ꕽ�ʏ�ɂ�����̂ɂ��邱�ƁD
		//! @param height �Z�p���̍����D
		//! @param color �F�Ddxlib��GetColor�Ŏ擾����.
		void DrawHexagonalPrism(const std::array<VECTOR, 6>& vertex, float height, unsigned int color);

	} // namespace dxlib_util

} // namespace designlab


#endif // !DESIGNLAB_DXLIB_H_