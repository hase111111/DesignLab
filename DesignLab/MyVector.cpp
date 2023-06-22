#include"MyVector.h"

using namespace my_vec;

bool my_vec::SVector::isZero() const
{
	// x,y,zともに絶対値が許容誤差以下のならばtrue
	if (my_math::isEqual(x,0.0f) && my_math::isEqual(y, 0.0f) && my_math::isEqual(z, 0.0f))
	{
				return true;
	}

	return false;
}

SVector& my_vec::SVector::operator+=(const SVector& _v)
{
	x += _v.x;
	y += _v.y;
	z += _v.z;
	return *this;
}

SVector& my_vec::SVector::operator-=(const SVector& _v)
{
	x -= _v.x;
	y -= _v.y;
	z -= _v.z;
	return *this;
}

SVector& my_vec::SVector::operator*=(const float _s)
{
	x *= _s;
	y *= _s;
	z *= _s;
	return *this;
}

SVector& my_vec::SVector::operator/=(const float _s)
{
	x /= _s;
	y /= _s;
	z /= _s;
	return *this;
}
