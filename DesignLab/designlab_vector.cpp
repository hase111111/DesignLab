#include"designlab_vector.h"


dl_vec::SVector dl_vec::SVector::normalized() const
{
	const float length = this->length();

	if (dl_math::isEqual(length, 0.0f))
	{
		return { 0.0f, 0.0f, 0.0f };
	}

	const float inv_length = 1.0f / length;
	return *this * inv_length;
}


bool dl_vec::SVector::isZero() const
{
	// x,y,zともに絶対値が許容誤差以下の値ならばtrue
	if (dl_math::isEqual(x, 0.0f) && dl_math::isEqual(y, 0.0f) && dl_math::isEqual(z, 0.0f))
	{
		return true;
	}

	return false;
}


dl_vec::SVector& dl_vec::SVector::operator+=(const SVector& other)
{
	x += other.x;
	y += other.y;
	z += other.z;
	return *this;
}


dl_vec::SVector& dl_vec::SVector::operator-=(const SVector& other)
{
	x -= other.x;
	y -= other.y;
	z -= other.z;
	return *this;
}


dl_vec::SVector& dl_vec::SVector::operator*=(const float other)
{
	x *= other;
	y *= other;
	z *= other;
	return *this;
}


dl_vec::SVector& dl_vec::SVector::operator/=(const float other)
{
	x /= other;
	y /= other;
	z /= other;
	return *this;
}
