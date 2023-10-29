//! @file designlab_euler.h
//! @brief �I�C���[�p��p������]��\���\����

#ifndef DESIGNLAB_EULER_H_
#define DESIGNLAB_EULER_H_


#include <string>

#include "designlab_math_util.h"
#include "designlab_vector3.h"


namespace designlab
{
	//! @struct designlab::EulerXYZ
	//! @brief ��]��\���\���́DXYZ�I�C���[�p�D�P�ʂ� [rad]
	//! @details XYZ�I�C���[�p�ɂ���ĉ�]��\������D
	//! @n ���[��(X��)�C�s�b�`(Y��)�C���[(Z��)�͂��ꂼ��E�˂��̕����ɉ�]����D
	//! @n
	//! @n �Q�l���� :  https://watako-lab.com/2019/01/23/roll_pitch_yaw/
	//! @n 
	//! @n �m���Ă̒ʂ�v�Z���Ԃ����Ȃ�|����̂ŁC���x���l����Ȃ�΃N�H�[�^�j�I��(�l����)��p������]�𐄏�����D
	struct EulerXYZ
	{
		constexpr EulerXYZ() : x_angle(0.f), y_angle(0.f), z_angle(0.f) {};
		constexpr EulerXYZ(const float x, const float y, const float z) : x_angle(x), y_angle(y), z_angle(z) {};
		constexpr EulerXYZ(const EulerXYZ& other) = default;
		constexpr EulerXYZ(EulerXYZ&& other) noexcept = default;
		constexpr EulerXYZ& operator =(const EulerXYZ& other) = default;
		
		~EulerXYZ() = default;

		constexpr EulerXYZ operator *(const float s) const noexcept { return { x_angle * s, y_angle * s, z_angle * s }; }


		constexpr bool operator ==(const EulerXYZ& other) const noexcept
		{
			return (
				::designlab::math_util::IsEqual(x_angle, other.x_angle) && 
				::designlab::math_util::IsEqual(y_angle, other.y_angle) &&
				::designlab::math_util::IsEqual(z_angle, other.z_angle)
			);
		}
		constexpr bool operator !=(const EulerXYZ& other) const noexcept { return !(*this == other); }


		//! @brief �I�C���[�p�� �P��[deg] �ŏ���������
		//! @param [in] x X������̉�]�D[deg]
		//! @param [in] y Y������̉�]�D[deg]
		//! @param [in] z Z������̉�]�D[deg]
		constexpr void SetDeg(const float x, const float y, const float z) 
		{
			x_angle = ::designlab::math_util::ConvertDegToRad(x);
			y_angle = ::designlab::math_util::ConvertDegToRad(y);
			z_angle = ::designlab::math_util::ConvertDegToRad(z);
		}

		//! @brief �I�C���[�p�𕶎���ɕϊ�����
		//! @n �P�ʂ� ���W�A�� [rad]
		//! @return std::string �I�C���[�p��\��������
		std::string ToString() const;

		//! @brief �I�C���[�p�𕶎���ɕϊ�����
		//! @n �P�ʂ� �x [deg]
		//! @return std::string �I�C���[�p��\��������
		std::string ToStringDeg() const;


		float x_angle;	//!< X ������̉�]�D[rad]
		float y_angle;	//!< Y ������̉�]�D[rad]
		float z_angle;	//!< Z ������̉�]�D[rad]
	};


	// �o�̓X�g���[��
	template <class Char>
	inline std::basic_ostream<Char>& operator <<(std::basic_ostream<Char>& os, const EulerXYZ& r)
	{
		os << ::designlab::math_util::ConvertFloatToString(r.x_angle) << Char(',') << 
			::designlab::math_util::ConvertFloatToString(r.y_angle) << Char(',') << 
			::designlab::math_util::ConvertFloatToString(r.z_angle);

		return os;
	}

	//���̓X�g���[��
	template <class Char>
	inline std::basic_istream<Char>& operator >>(std::basic_istream<Char>& is, EulerXYZ& r)
	{
		Char unused;
		return is >> unused >> r.x_angle >> unused >> r.y_angle >> unused >> r.z_angle >> unused;
	}


	//! @brief ��]�������x�N�g����Ԃ��D�O�p�֐��̏����������d�����̂Œ��ӁD
	//! @param [in] vec �ʒu�x�N�g��
	//! @param [in] rot ��]�x�N�g��
	//! @return Vector3 ��]������̈ʒu�x�N�g��
	Vector3 RotateVector3(const Vector3& vec, const EulerXYZ& rot);

}	// namespace designlab


#endif // !DESIGNLAB_EULER_H_