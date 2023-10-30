//! @file designlab_rotation_matrix.h
//! @brief ��]�s���\���\����

#ifndef DESIGNLAB_ROTATION_MATRIX_H_
#define DESIGNLAB_ROTATION_MATRIX_H_


#include <array>

#include "designlab_euler.h"
#include "designlab_vector3.h"


namespace designlab 
{
	//! @struct designlab::RotationMatrix3x3
	//! @brief 3�����̉�]�s���\���\����
	//! 	@details ��]�s��ɂ���
	//! @n https://w3e.kanazawa-it.ac.jp/math/category/gyouretu/senkeidaisu/henkan-tex.cgi?target=/math/category/gyouretu/senkeidaisu/rotation_matrix.html
	//! @n https://programming-surgeon.com/script/euler-angle/
	struct RotationMatrix3x3 final
	{
		//! @brief �P�ʍs��𐶐�����
		RotationMatrix3x3() : element({ {
			{ 1.0f, 0.0f, 0.0f },
			{ 0.0f, 1.0f, 0.0f },
			{ 0.0f, 0.0f, 1.0f }
		} }) {};

		//! @brief �C�ӂ̉�]�s��𐶐�����
		RotationMatrix3x3(
			const float r11, const float r12, const float r13,
			const float r21, const float r22, const float r23,
			const float r31, const float r32, const float r33
		) : element({ {
			{ r11, r12, r13 },
			{ r21, r22, r23 },
			{ r31, r32, r33 }
		} }) {};

		RotationMatrix3x3(const RotationMatrix3x3& other) = default;
		RotationMatrix3x3(RotationMatrix3x3&& other) noexcept = default;
		RotationMatrix3x3& operator =(const RotationMatrix3x3& other) = default;
		~RotationMatrix3x3() = default;

		RotationMatrix3x3 operator* (const RotationMatrix3x3& other) const;


		//! @brief ��]�s���XYZ�I�C���[�p�ɕϊ�����D
		//! @return XYZ�I�C���[�p
		EulerXYZ ToEulerXYZ() const;

		//! @brief x������ɉ�]�����]�s��𐶐�����
		//! @param [in] angle ��]�p [rad]
		//! @return x������ɉ�]�����]�s��
		static [[nodiscard]] RotationMatrix3x3 CreateRotationMatrixX(float angle);

		//! @brief y������ɉ�]�����]�s��𐶐�����
		//! @param [in] angle ��]�p [rad]
		//! @return y������ɉ�]�����]�s��
		static [[nodiscard]] RotationMatrix3x3 CreateRotationMatrixY(float angle);

		//! @brief z������ɉ�]�����]�s��𐶐�����
		//! @param [in] angle ��]�p [rad]
		//! @return z������ɉ�]�����]�s��
		static [[nodiscard]] RotationMatrix3x3 CreateRotationMatrixZ(float angle);


		//! @brief ��]�s��𕶎���ɕϊ�����D
		//! @return std::string ��]�s���\��������D
		[[nodiscard]] std::string ToString() const;
		

		//! �f�[�^�̕��тɂ���
		//! @n | R11 R12 R13 |
		//! @n | R21 R22 R23 |   
		//! @n | R31 R32 R33 |
		//! @n
		//! @n R11��element[0][0]�CR12��element[0][1]�CR32��element[2][1]�ƂȂ�D
		//! @n �܂�Celement[ �s - 1 ][ �� - 1 ]�ƂȂ�D
		std::array<std::array<float,3>, 3> element;
	};


	//! @brief ��]�������x�N�g����Ԃ�
	//! @param [in] vec ��]������x�N�g��
	//! @param [in] rot ��]�s��
	Vector3 RotateVector3(const Vector3& vec, const RotationMatrix3x3& rot);

}


#endif	// !DESIGNLAB_ROTATION_MATRIX_H_