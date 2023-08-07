#pragma once

#include <fstream>
#include <iostream>
#include <iomanip>
#include <ios>

#include "my_math.h"
#include "my_vector.h"


namespace my_vec
{
	//! @struct my_vec::SRotator
	//! @date 2023/08/06
	//! @author ���J��
	//! @brief ��]��\���\���́DXYZ�I�C���[�p
	//! @details XYZ�I�C���[�p�ɂ���ĉ�]��\������D
	//! @n ���[��(X��)�C�s�b�`(Y��)�C���[(Z��)�͂��ꂼ��E�˂��̕����ɉ�]����D@n
	//! @n �Q�l���� :  https://watako-lab.com/2019/01/23/roll_pitch_yaw/
	struct SRotator
	{
		SRotator() = default;
		constexpr SRotator(const float r, const float p, const float y) : roll(r), pitch(p), yaw(y) {};


		constexpr SRotator operator *(const float s) const { return { roll * s, pitch * s, yaw * s }; }


		constexpr bool operator ==(const SRotator& other) const
		{
			return (my_math::isEqual(roll, other.roll) && my_math::isEqual(pitch, other.pitch) && my_math::isEqual(yaw, other.yaw));
		}
		constexpr bool operator !=(const SRotator& other) const { return !(*this == other); }


		float roll;		//!< ���[���DX������̉�]
		float pitch;	//!< �s�b�`�DY������̉�]
		float yaw;		//!< ���[�DZ������̉�]
	};


	// �o�̓X�g���[��
	inline std::ostream& operator<<(std::ostream& os, const SRotator& r)
	{
		const int width = 10;
		const char fill = ' ';
		const int precision = 3;

		os << std::fixed << std::setprecision(precision);
		os << "(roll : ";
		os << std::setw(width) << std::setfill(fill) << r.roll;
		os << "[rad], pitch : ";
		os << std::setw(width) << std::setfill(fill) << r.pitch;
		os << "[rad], yaw : ";
		os << std::setw(width) << std::setfill(fill) << r.yaw;
		os << "[rad])";
		return os;
	}


	//���̓X�g���[��
	template <class Char>
	inline std::basic_istream<Char>& operator >>(std::basic_istream<Char>& is, SRotator& r)
	{
		Char unused;
		return is >> unused >> r.roll >> unused >> r.pitch >> unused >> r.yaw >> unused;
	}

	//! @brief ��]�������x�N�g����Ԃ��D�O�p�֐��̏����������d�����̂Œ��ӁD
	//! @param [in] vec �ʒu�x�N�g��
	//! @param [in] rot ��]�x�N�g��
	//! @return SVector ��]������̈ʒu�x�N�g��
	SVector rotVector(const SVector& vec, const SRotator& rot);

}	// namespace my_vec


//! @file my_rotator.h
//! @date 2023/08/06
//! @auther ���J��
//! @brief ��]��\���\���́DXYZ�I�C���[�p
//! @n �s�� : @lineinfo