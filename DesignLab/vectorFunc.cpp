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

SVector& myvector::SVector::operator*=(const double _s)
{
	x *= _s;
	y *= _s;
	z *= _s;
	return *this;
}

SVector& myvector::SVector::operator/=(const double _s)
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

bool myvector::isEqualVector(const SVector& In1, const SVector& In2, const double _error)
{
	if ((abs(In1.x - In2.x) < _error) && (abs(In1.y - In2.y) < _error) && (abs(In1.z - In2.z) < _error))
	{
		return true;
	}

	return false;
}

bool myvector::isLineSegmentHit(const SVector& _s1, const SVector& _e1, const SVector& _s2, const SVector& _e2, SVector& _cross_point)
{
	myvector::SVector _v1 = myvector::subVec(_e1, _s1);		//線分1の方向ベクトル
	myvector::SVector _v2 = myvector::subVec(_e2, _s2);		//線分2の方向ベクトル
	myvector::SVector _v = myvector::subVec(_s2, _s1);		//線分1の始点から線分2の始点へ進むベクトル

	// 方向ベクトルの外積を求めて，それが０か確かめる．許容誤差を設けている．０ならば平行なので交点はない
	if (myvector::getVecCrossXY(_v1, _v2) < Define::ALLOWABLE_ERROR)
	{
		return false;
	}

	// s1 + v1 * t1 が交点となるような内分点.詳しくは http://marupeke296.com/COL_2D_No10_SegmentAndSegment.html を見て...
	double _t1 = myvector::getVecCrossXY(_v, _v2) / myvector::getVecCrossXY(_v1, _v2);
	double _t2 = myvector::getVecCrossXY(_v, _v1) / myvector::getVecCrossXY(_v1, _v2);

	//交点が存在するか調べる． 0 < t1 < 1　かつ 0 < t2 < 1 である必要がある
	if (_t1 + Define::ALLOWABLE_ERROR < 0 || _t1 - Define::ALLOWABLE_ERROR > 1)
	{
		// t1について条件を満たさない
		return false;
	}

	if (_t2 + Define::ALLOWABLE_ERROR < 0 || _t2 - Define::ALLOWABLE_ERROR > 1)
	{
		// t2について条件を満たさない
		return false;
	}

	//上記のどちらにも該当しないなら接触している！よって交点を算出する．
	_cross_point = myvector::addVec(_s1, myvector::VScale(_v1, _t1));

	return true;
}

