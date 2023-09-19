#include"designlab_vector3.h"

namespace designlab 
{

	Vector3 Vector3::normalized() const
	{
		const float length = this->length();

		if (dl_math::isEqual(length, 0.0f))
		{
			return { 0.0f, 0.0f, 0.0f };
		}

		// 割り算は遅いので、逆数をかける
		const float inv_length = 1.0f / length;
		return *this * inv_length;
	}

	bool Vector3::isZero() const
	{
		// x,y,zともに絶対値が許容誤差以下の値ならばtrue
		if (dl_math::isEqual(x, 0.0f) && dl_math::isEqual(y, 0.0f) && dl_math::isEqual(z, 0.0f))
		{
			return true;
		}

		return false;
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








