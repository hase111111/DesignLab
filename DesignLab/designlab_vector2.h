//! @file designlab_vector2.h
//! @brief 2�����̃x�N�g����\���\����

#ifndef DESIGNLAB_VECTOR2_H_
#define DESIGNLAB_VECTOR2_H_


#include <cmath>
#include <iostream>
#include <string>

#include "designlab_math_util.h"


namespace designlab
{
	//! @struct designlab::SVector2
	//! @brief 2�����̈ʒu�x�N�g����\���\����
	//! @details �Q�l : https://qiita.com/Reputeless/items/96226cfe1282a014b147
	//! @n �{���̓e���v���[�g�\���̂ɂ���ׂ������ǁC�߂�ǂ���...�^���Ԃ�float�̂܂܂����Ȃ���.... 
	struct SVector2 final
	{
		constexpr SVector2() : x(0), y(0) {};
		constexpr SVector2(float x_pos, float y_pos) : x(x_pos), y(y_pos) {}
		constexpr SVector2(const SVector2& other) = default;
		constexpr SVector2(SVector2&& other) noexcept = default;
		constexpr SVector2& operator =(const SVector2& other) = default;

		constexpr SVector2 operator +() const { return *this; }
		constexpr SVector2 operator -() const { return{ -x, -y }; }
		constexpr SVector2 operator +(const SVector2& other) const { return{ x + other.x, y + other.y }; }
		constexpr SVector2 operator +(const float s) const { return{ x + s, y + s }; }
		constexpr SVector2 operator -(const SVector2& other) const { return{ x - other.x, y - other.y }; }
		constexpr SVector2 operator -(const float s) const { return{ x - s, y - s }; }
		constexpr SVector2 operator *(float s) const { return{ x * s, y * s }; }
		constexpr SVector2 operator /(float s) const { return{ x / s, y / s }; }

		SVector2& operator +=(const SVector2& other);
		SVector2& operator -=(const SVector2& other);
		SVector2& operator *=(float s);
		SVector2& operator /=(float s);

		constexpr bool operator==(const SVector2& other) const
		{
			return ::designlab::math_util::IsEqual(x, other.x) && ::designlab::math_util::IsEqual(y, other.y);
		}


		//! @brief ���̃x�N�g���̒�����Ԃ�
		//! @return ���̃x�N�g���̒���
		//! @note sqrt�͏d���̂ŁC������2���Ԃ�lengthSquare()���g�����Ƃ𐄏�
		float Length() const { return std::sqrt(LengthSquare()); }

		//! @brief ���̃x�N�g���̒�����2���Ԃ�
		//! @return ���̃x�N�g���̒�����2��
		constexpr float LengthSquare() const { return Dot(*this); }

		//! @brief ���̃x�N�g����other�̓��ς�Ԃ�
		//! @param [in] other ���̃x�N�g��
		//! @return ���̃x�N�g����other�̓���
		constexpr float Dot(const SVector2& other) const { return x * other.x + y * other.y; }

		//! @brief ���̃x�N�g����other�̊O�ς�Ԃ�
		//! @param [in] other ���̃x�N�g��
		//! @return ���̃x�N�g����other�̊O��
		//! @note 2�����Ȃ̂ŁC�O�ς̓X�J���[
		constexpr float Cross(const SVector2& other) const { return x * other.y - y * other.x; }

		//! @brief ���̃x�N�g����other�̋�����Ԃ�
		//! @param [in] other ���̃x�N�g��
		//! @return ���̃x�N�g����other�̋���
		float DistanceFrom(const SVector2& other) const { return (other - *this).Length(); }

		//! @brief ���̃x�N�g���𐳋K�������x�N�g����Ԃ�
		//! @return ���K�����ꂽ�x�N�g��
		SVector2 Normalized() const;

		//! @brief ���̃x�N�g����0�Ȃ��true
		//! @return ���̃x�N�g����0�Ȃ��true
		//! @note �덷���l�����Ă���
		constexpr bool IsZero() const { return ::designlab::math_util::IsEqual(x, 0.0f) && ::designlab::math_util::IsEqual(y, 0.0f); }


		//! @brief ���̃x�N�g���𕶎���ɂ��ĕԂ�
		//! @n (x, y) �̌`���C�����_�ȉ�3���܂�
		//! @return ���̃x�N�g���𕶎���ɂ�������
		std::string ToString() const;


		float x;
		float y;
	};


	inline constexpr SVector2 operator *(float s, const SVector2& v)
	{
		return{ s * v.x, s * v.y };
	}


	template <class Char>
	inline std::basic_ostream<Char>& operator <<(std::basic_ostream<Char>& os, const SVector2& v)
	{
		return os << Char('(') << v.x << Char(',') << v.y << Char(')');
	}


	template <class Char>
	inline std::basic_istream<Char>& operator >>(std::basic_istream<Char>& is, SVector2& v)
	{
		Char unused;
		return is >> unused >> v.x >> unused >> v.y >> unused;
	}

} // namespace designlab


#endif	