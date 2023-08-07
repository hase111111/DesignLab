#include"my_vector.h"


bool my_vec::SVector::isZero() const
{
	// x,y,zともに絶対値が許容誤差以下の値ならばtrue
	if (my_math::isEqual(x, 0.0f) && my_math::isEqual(y, 0.0f) && my_math::isEqual(z, 0.0f))
	{
		return true;
	}

	return false;
}

my_vec::SVector& my_vec::SVector::operator+=(const SVector& other)
{
	x += other.x;
	y += other.y;
	z += other.z;
	return *this;
}

my_vec::SVector& my_vec::SVector::operator-=(const SVector& other)
{
	x -= other.x;
	y -= other.y;
	z -= other.z;
	return *this;
}

my_vec::SVector& my_vec::SVector::operator*=(const float other)
{
	x *= other;
	y *= other;
	z *= other;
	return *this;
}

my_vec::SVector& my_vec::SVector::operator/=(const float other)
{
	x /= other;
	y /= other;
	z /= other;
	return *this;
}
