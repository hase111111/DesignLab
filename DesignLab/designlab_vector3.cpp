#include "designlab_vector3.h"

#include "designlab_math_util.h"


namespace dl = ::designlab;
namespace dlm = ::designlab::math_util;


dl::Vector3 dl::Vector3::GetNormalized() const noexcept
{
	const float GetLength = this->GetLength();

	if (dlm::IsEqual(GetLength, 0.0f))
	{
		return { 0.0f, 0.0f, 0.0f };
	}

	// 割り算は遅いので、逆数をかける
	const float inv_length = 1.0f / GetLength;
	return *this * inv_length;
}

bool dl::Vector3::IsZero() const noexcept
{
	//_ x,y,zともに絶対値が許容誤差以下の値ならばtrue
	if (dlm::IsEqual(x, 0.0f) && dlm::IsEqual(y, 0.0f) && dlm::IsEqual(z, 0.0f))
	{
		return true;
	}

	return false;
}

std::string dl::Vector3::ToString() const
{
	return std::string("( x : ") + dlm::ConvertFloatToString(x) +
		std::string(", y : ") + dlm::ConvertFloatToString(y) +
		std::string(", z : ") + dlm::ConvertFloatToString(z) + std::string(")");
}

std::string dl::Vector3::ToCsvString() const
{
	return dlm::ConvertFloatToString(x) + std::string(",") + dlm::ConvertFloatToString(y) + std::string(",") + dlm::ConvertFloatToString(z);
}

dl::Vector3& dl::Vector3::operator+=(const Vector3& other) noexcept
{
	x += other.x;
	y += other.y;
	z += other.z;
	return *this;
}

dl::Vector3& dl::Vector3::operator-=(const Vector3& other) noexcept
{
	x -= other.x;
	y -= other.y;
	z -= other.z;
	return *this;
}

dl::Vector3& dl::Vector3::operator*=(const float other) noexcept
{
	x *= other;
	y *= other;
	z *= other;
	return *this;
}

dl::Vector3& dl::Vector3::operator/=(const float other)
{
	x /= other;
	y /= other;
	z /= other;
	return *this;
}