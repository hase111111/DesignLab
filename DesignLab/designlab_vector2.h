//! @file designlab_vector2.h
//! @brief 2�����̈ʒu�x�N�g����\���\����

#ifndef DESIGNLAB_VECTOR2_H_
#define DESIGNLAB_VECTOR2_H_


#include <cmath>
#include <string>

#include "designlab_math_util.h"


namespace designlab
{
	//! @struct designlab::Vector2
	//! @brief 2�����̈ʒu�x�N�g����\���\����
	//! @details �Q�l : https://qiita.com/Reputeless/items/96226cfe1282a014b147
	//! @n �{���̓e���v���[�g�\���̂ɂ���ׂ������ǁC�߂�ǂ���...�^���Ԃ�float�̂܂܂����Ȃ���.... 
	struct Vector2 final
	{
		constexpr Vector2() : x(0), y(0) {};
		constexpr Vector2(float x_pos, float y_pos) : x(x_pos), y(y_pos) {}
		constexpr Vector2(const Vector2& other) = default;
		constexpr Vector2(Vector2&& other) noexcept = default;
		constexpr Vector2& operator =(const Vector2& other) = default;

		constexpr Vector2 operator +() const { return *this; }
		constexpr Vector2 operator -() const { return{ -x, -y }; }
		constexpr Vector2 operator +(const Vector2& other) const { return{ x + other.x, y + other.y }; }
		constexpr Vector2 operator -(const Vector2& other) const { return{ x - other.x, y - other.y }; }
		constexpr Vector2 operator *(float s) const { return{ x * s, y * s }; }
		constexpr Vector2 operator /(float s) const { return{ x / s, y / s }; }

		Vector2& operator +=(const Vector2& other);
		Vector2& operator -=(const Vector2& other);
		Vector2& operator *=(float s);
		Vector2& operator /=(float s);

		constexpr bool operator==(const Vector2& other) const
		{
			return ::designlab::math_util::IsEqual(x, other.x) && ::designlab::math_util::IsEqual(y, other.y);
		}

		constexpr bool operator!=(const Vector2& other) const { return !(*this == other); }


		//! @brief ���̃x�N�g���̒�����Ԃ�
		//! @return ���̃x�N�g���̒���
		//! @note sqrt�͏d���̂ŁC������2���Ԃ�lengthSquare()���g�����Ƃ𐄏�
		float GetLength() const { return std::sqrt(GetSquaredLength()); }

		//! @brief ���̃x�N�g���̒�����2���Ԃ�
		//! @return ���̃x�N�g���̒�����2��
		constexpr float GetSquaredLength() const noexcept { return Dot(*this); }

		//! @brief ���̃x�N�g����other�̓��ς�Ԃ�
		//! @param [in] other ���̃x�N�g��
		//! @return ���̃x�N�g����other�̓���
		constexpr float Dot(const Vector2& other) const noexcept { return x * other.x + y * other.y; }

		//! @brief ���̃x�N�g����other�̊O�ς�Ԃ�
		//! @param [in] other ���̃x�N�g��
		//! @return ���̃x�N�g����other�̊O��
		//! @note 2�����Ȃ̂ŁC�O�ς̓X�J���[
		constexpr float Cross(const Vector2& other) const noexcept { return x * other.y - y * other.x; }

		//! @brief ���̃x�N�g����other�̋�����Ԃ�
		//! @param [in] other ���̃x�N�g��
		//! @return ���̃x�N�g����other�̋���
		float GetDistanceFrom(const Vector2& other) const noexcept { return (other - *this).GetLength(); }

		//! @brief ���̃x�N�g���𐳋K�������x�N�g����Ԃ�
		//! @return ���K�����ꂽ�x�N�g��
		Vector2 GetNormalized() const;

		//! @brief ���̃x�N�g����0�Ȃ��true
		//! @return ���̃x�N�g����0�Ȃ��true
		//! @note �덷���l�����Ă���
		constexpr bool IsZero() const noexcept{ return ::designlab::math_util::IsEqual(x, 0.0f) && ::designlab::math_util::IsEqual(y, 0.0f); }


		//! @brief ���̃x�N�g���𕶎���ɂ��ĕԂ�
		//! @n (x, y) �̌`���C�����_�ȉ�3���܂�
		//! @return ���̃x�N�g���𕶎���ɂ�������
		std::string ToString() const;


		float x;
		float y;
	};


	constexpr Vector2 operator *(float s, const Vector2& v)
	{
		return { s * v.x, s * v.y };
	}

	template <class Char>
	std::basic_ostream<Char>& operator <<(std::basic_ostream<Char>& os, const Vector2& v)
	{
		return os << ::designlab::math_util::ConvertFloatToString(v.x) << Char(',') << ::designlab::math_util::ConvertFloatToString(v.y);
	}

	template <class Char>
	inline std::basic_istream<Char>& operator >>(std::basic_istream<Char>& is, Vector2& v)
	{
		Char unused;
		return is >> unused >> v.x >> unused >> v.y >> unused;
	}

} // namespace designlab


#endif	// DESIGNLAB_VECTOR2_H_