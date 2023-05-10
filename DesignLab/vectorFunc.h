#pragma once
#include "Define.h"
#include <cmath>
#include <fstream>

//
//	↓先輩がおそらく衝突判定の参考資料にしたであろうサイト
//	http://marupeke296.com/COL_main.html
//

namespace myvector 
{
	// ベクトルデータ型
	struct SVector
	{
		double x, y, z;
	};
	 
	//脚座標用のベクトルデータ型．6つの脚それぞれの座標を入れる
	struct SLegVector 
	{
		SVector leg[Define::LEG_NUM];
	};

	bool isEqualVector(const SVector& In1, const SVector& In2);

	__inline SVector VGet(const double x, const double y, const double z)
	{
		SVector Result;
		Result.x = x;
		Result.y = y;
		Result.z = z;
		return Result;
	}

	// ベクトルの加算
	__inline SVector addVec(const SVector &In1, const SVector &In2)
	{
		SVector Result;
		Result.x = In1.x + In2.x;
		Result.y = In1.y + In2.y;
		Result.z = In1.z + In2.z;
		return Result;
	}

	// ベクトルの減算
	__inline SVector subVec(const SVector &In1, const SVector &In2)
	{
		SVector Result;
		Result.x = In1.x - In2.x;
		Result.y = In1.y - In2.y;
		Result.z = In1.z - In2.z;
		return Result;
	}
	// ベクトルの内積
	__inline double VDot(const SVector &In1, const SVector &In2)
	{
		return In1.x * In2.x + In1.y * In2.y + In1.z * In2.z;
	}

	// ベクトルの外積
	__inline SVector VCross(const SVector &In1, const SVector &In2)
	{
		SVector Result;
		Result.x = In1.y * In2.z - In1.z * In2.y;
		Result.y = In1.z * In2.x - In1.x * In2.z;
		Result.z = In1.x * In2.y - In1.y * In2.x;
		return Result;
	}

	// ベクトルのスケーリング
	__inline SVector VScale(const SVector &In, double Scale)
	{
		SVector Result;
		Result.x = In.x * Scale;
		Result.y = In.y * Scale;
		Result.z = In.z * Scale;
		return Result;
	}

	// ベクトルのサイズの２乗
	__inline double VSquareSize(const SVector &In)
	{
		return In.x * In.x + In.y * In.y + In.z * In.z;
	}

	// ベクトルの大きさ
	__inline double VMag(const SVector &In1) 
	{
		return sqrt(VDot(In1, In1));
	}

	// ベクトルの大きさを１に
	__inline SVector  VUnit(const SVector &In1) 
	{
		return VScale(In1, 1 / VMag(In1));
	}

	// ベクトルの大きさ 位置ベクトルから
	__inline double VMag2(const SVector &In1, const SVector &In2) 
	{
		return VMag(subVec(In1, In2));
	}

	//ベクトルの出力
	void VectorOutPut(SVector v1, std::ofstream& fout);
	void VectorOutPut(SVector v1);

	////////////////////////////////////////////////////////////////////////////
	//座標系はmainの座標系でPRYそれぞれに角度を入れたときの回転は
	//VRot(myvector::VECTOR In, myvector::VECTOR center, double thP, double thR, double thY)
	//Y(z軸左回り)→P(x軸左回り)→R(y軸左回り)
	//VRot(const myvector::VECTOR &In, double thP, double thR,double thY )
	//Y(z軸右回り)→P(y軸右回り)→R(x軸右回り)
	//VRot_R(myvector::VECTOR In, myvector::VECTOR center, double thP, double thR, double thY)
	//R(y軸左回り)→P(x軸左回り)→Y(z軸左回り)
	///////////////////////////////////////////////////////////////////////////
	//ベクトルの3Dローテーション オイラー角（テイト・ブライアン角）Y-X-Z
	__inline SVector VRot(const SVector In, const SVector center, const double thP, const double thR, const double thY) 
	{
		SVector ans, buf;
		buf = subVec(In, center);
		ans.x = (cos(thR) * cos(thY) + sin(thR) * sin(thP) * sin(thY)) * buf.x +
			(cos(thY) * sin(thR) * sin(thP) - cos(thR) * sin(thY)) * buf.y +
			(cos(thP) * sin(thR)) * buf.z;

		ans.y = cos(thP) * sin(thY) * buf.x +
			cos(thP) * cos(thY) * buf.y +
			(-sin(thP))        * buf.z;

		ans.z = (cos(thR) * sin(thP) * sin(thY) - cos(thY) * sin(thR)) * buf.x +
			(cos(thR) * cos(thY) * sin(thP) + sin(thR) * sin(thY)) * buf.y +
			(cos(thR) * cos(thP)) * buf.z;

		return myvector::addVec(ans, center);
	}

	//上の関数と統合する予定だったけど...
	__inline SVector VRot(const SVector &In, const double thP, const double thR, const double thY)
	{
		SVector ans;
		ans.y = (cos(thR) * cos(thY) + sin(thR) * sin(thP) * sin(thY)) * In.y +
			(cos(thY) * sin(thR) * sin(thP) - cos(thR) * sin(thY)) * In.x +
			(cos(thP) * sin(thR)) * In.z;

		ans.x = cos(thP) * sin(thY) * In.y +
			cos(thP) * cos(thY) * In.x +
			(-sin(thP))        * In.z;

		ans.z = (cos(thR) * sin(thP) * sin(thY) - cos(thY) * sin(thR)) * In.y +
			(cos(thR) * cos(thY) * sin(thP) + sin(thR) * sin(thY)) * In.x +
			(cos(thR) * cos(thP)) * In.z;
		return ans;
	}

	//ベクトルの3D逆ローテーション 上の関数の逆変換//R→P→Y
	__inline SVector VRot_R(SVector In, SVector center, const double thP, const double thR, const double thY) 
	{

		SVector ans, buf;
		buf = myvector::subVec(In, center);
		ans.x = (cos(thY) * cos(thR) - sin(thY) * sin(thP) * sin(thR)) * buf.x +
			(-cos(thP) * sin(thY)) * buf.y +
			(cos(thR) * sin(thY) * sin(thP) + cos(thY) * sin(thR)) * buf.z;

		ans.y = (cos(thY) * sin(thP) * sin(thR) + cos(thR) * sin(thY)) * buf.x +
			(cos(thY) * cos(thP)) * buf.y +
			(sin(thY) * sin(thR) - cos(thY) * cos(thR) * sin(thP)) * buf.z;


		ans.z = -cos(thP) * sin(thR)	* buf.x +
			sin(thP)				* buf.y +
			cos(thP) * cos(thR)	* buf.z;
		return myvector::addVec(ans, center);
	}

	//2次元用
	// ベクトルの内積
	__inline double V2Dot(const SVector &In1, const SVector &In2)
	{
		return In1.x * In2.x + In1.y * In2.y;
	}

	// 【2次元用】ベクトルの外積を得る関数．Zの値を無視して計算する
	__inline double	getVecCrossXY(const SVector &In1, const SVector &In2)
	{
		return In1.x * In2.y - In1.y * In2.x;
	}

	// ベクトルのサイズの２乗
	__inline double	V2SquareSize(const SVector &In)
	{
		return In.x * In.x + In.y * In.y;
	}

	// ベクトルの大きさ
	__inline double V2Mag(const SVector &In1) 
	{
		return sqrt(V2Dot(In1, In1));
	}

	// ベクトルの大きさを１に
	__inline SVector  V2Unit(const SVector &In1) 
	{
		return VScale(In1, 1 / V2Mag(In1));
	}

	// ベクトルの大きさ 位置ベクトルから
	__inline double V2Mag2(const SVector &In1, const SVector &In2) 
	{
		return V2Mag(subVec(In1, In2));
	}

	// X-Y平面における線分の接触判定．接触しているならばtrueを返す．LineSegmentは線分のこと
	// s1 :	線分1の始点 e1 : 線分1の終点 s2 : 線分2の始点 e2 : 線分2の終点 crossPoint: 交点の座標を参照渡しする．falseの時変更されない
	bool isLineSegmentHit(const SVector& _s1, const SVector& _e1, const SVector& _s2, const SVector& _e2, SVector& _cross_point);
}
