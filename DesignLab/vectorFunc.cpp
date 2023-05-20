#include"vectorFunc.h"
#include <iostream>

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


void myvector::VectorOutPut(SVector v1, std::ofstream& fout) 
{
	fout << "( " << v1.x << "," << v1.y << "," << v1.z << " )\n";
}

void myvector::VectorOutPut(SVector v1) 
{
	std::cout << "( " << (int)v1.x << "," << (int)v1.y << "," << v1.z << " )\n";
}

bool myvector::isEqualVector(const SVector& In1, const SVector& In2, const float _error)
{
	if ((abs(In1.x - In2.x) < _error) && (abs(In1.y - In2.y) < _error) && (abs(In1.z - In2.z) < _error))
	{
		return true;
	}

	return false;
}

