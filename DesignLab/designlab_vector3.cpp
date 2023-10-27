#include "designlab_vector3.h"

#include "designlab_math_util.h"

namespace dlm = designlab::math_util;


namespace designlab 
{

	Vector3 Vector3::GetNormalized() const
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

	bool Vector3::IsZero() const
	{
		//_ x,y,zともに絶対値が許容誤差以下の値ならばtrue
		if (dlm::IsEqual(x, 0.0f) && dlm::IsEqual(y, 0.0f) && dlm::IsEqual(z, 0.0f))
		{
			return true;
		}

		return false;
	}

	std::string Vector3::ToString() const
	{
		return std::string("( x : ") + dlm::ConvertFloatToString(x) + 
			std::string(", y : ") + dlm::ConvertFloatToString(y) + 
			std::string(", z : ") + dlm::ConvertFloatToString(z) + std::string(")");
	}

	std::string Vector3::ToCsvString() const
	{
		return dlm::ConvertFloatToString(x) + std::string(",") + dlm::ConvertFloatToString(y) + std::string(",") + dlm::ConvertFloatToString(z);
	}

	Vector3& Vector3::operator+=(const Vector3& other)
	{
		x += other.x;
		y += other.y;
		z += other.z;
		return *this;
	}

	Vector3& Vector3::operator-=(const Vector3& other)
	{
		x -= other.x;
		y -= other.y;
		z -= other.z;
		return *this;
	}

	Vector3& Vector3::operator*=(const float other)
	{
		x *= other;
		y *= other;
		z *= other;
		return *this;
	}

	Vector3& Vector3::operator/=(const float other)
	{
		x /= other;
		y /= other;
		z /= other;
		return *this;
	}

}	// namespace designlab