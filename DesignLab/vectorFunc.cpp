#include"vectorFunc.h"

using namespace myvector;

bool myvector::SVector::isZero() const
{
	// x,y,zともに絶対値が許容誤差以下のならばtrue
	if (-Define::ALLOWABLE_ERROR < x && x < Define::ALLOWABLE_ERROR) 
	{
		if (-Define::ALLOWABLE_ERROR < y && y < Define::ALLOWABLE_ERROR)
		{
			if (-Define::ALLOWABLE_ERROR < z && z < Define::ALLOWABLE_ERROR)
			{
				return true;
			}
		}
	}

	return false;
}

SVector& myvector::SVector::operator+=(const SVector& _v)
{
	x += _v.x;
	y += _v.y;
	z += _v.z;
	return *this;
}

SVector& myvector::SVector::operator-=(const SVector& _v)
{
	x -= _v.x;
	y -= _v.y;
	z -= _v.z;
	return *this;
}

SVector& myvector::SVector::operator*=(const float _s)
{
	x *= _s;
	y *= _s;
	z *= _s;
	return *this;
}

SVector& myvector::SVector::operator/=(const float _s)
{
	x /= _s;
	y /= _s;
	z /= _s;
	return *this;
}
