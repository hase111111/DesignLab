#pragma once
#include "HexapodConst.h"
#include "Define.h"
#include <cmath>
#include <fstream>
#include <iostream>

namespace myvector 
{
	// ベクトルデータ型
	// 参考 https://qiita.com/Reputeless/items/96226cfe1282a014b147
	//		https://qiita.com/KRiver1/items/ef7731467b5ca83850cb
	//		https://atcoder.jp/contests/apg4b/tasks/APG4b_ab?lang=ja 「細かい話」のコンストラクタの項
	//		https://programming.pc-note.net/cpp/operator2.html		比較演算子
	//		http://marupeke296.com/COL_main.html  ←先輩がおそらく衝突判定の参考資料にしたであろうサイト
	//
	//	ヘッダファイル内に実装を書くのは個人的には避けたいのだが，constexpr関数を使う場合，このようにしたほうが良いらしい．
	//	実行速度が大切なプログラムであるため，このように処理を記述する．
	//
	struct SVector
	{
		SVector() = default;
		constexpr SVector(const double _x, const double _y, const double _z) : x(_x), y(_y), z(_z) {};	
		constexpr SVector(const SVector& other) : x(other.x), y(other.y), z(other.z) {};				// コピーコンストラクタ

		double x, y, z;

		//演算用メンバ変数
		constexpr double lengthSquare() const { return x * x + y * y + z * z; }	// x,y,zの値を2乗にして足し合わせたスカラー値を出力する．
		double length() const { return std::sqrt(lengthSquare()); }				// ベクトルの長さをスカラー値で出力する．
		SVector normalized() const { return *this / length(); }					// 単位ベクトルを返す．normalizeとは，ベクトルを正規化（単位ベクトルに変換）する操作を表す．
		bool isZero() const;													// x,y,zともに絶対値が許容誤差以下のならばtrueを返す．

		//関係演算子、等価演算子
		constexpr bool operator == (const SVector& _v) const 
		{
			if (_v.x - Define::ALLOWABLE_ERROR < x && x < _v.x + Define::ALLOWABLE_ERROR) 
			{
				if (_v.y - Define::ALLOWABLE_ERROR < y && y < _v.y + Define::ALLOWABLE_ERROR)
				{
					if (_v.z - Define::ALLOWABLE_ERROR < z && z < _v.z + Define::ALLOWABLE_ERROR) { return true; }
				}
			}
			return false;
		}
		constexpr bool operator != (const SVector& _v) const { return !(*this == _v); }
		bool operator < (const SVector& _v) const { return length() < _v.length(); }
		bool operator > (const SVector& _v) const { return _v < *this; }
		bool operator <= (const SVector& _v) const { return !(*this > _v); }
		bool operator >= (const SVector& _v) const { return !(*this < _v); }

		//算術演算子
		constexpr SVector operator +() const { return *this; }
		constexpr SVector operator -() const { return { -x, -y, -z }; }
		constexpr SVector operator +(const SVector& _v) const { return { x + _v.x , y + _v.y, z + _v.z }; }
		constexpr SVector operator -(const SVector& _v) const { return { x - _v.x , y - _v.y, z - _v.z }; }
		SVector& operator += (const SVector& _v);
		SVector& operator -= (const SVector& _v);
		constexpr SVector operator *(const double _s) const { return { x * _s, y * _s, z * _s }; }
		constexpr SVector operator /(const double _s) const { return { x / _s, y / _s, z / _s }; }
		SVector& operator *= (const double _s);
		SVector& operator /= (const double _s);
	};
	
	// スカラーが先に来る場合の掛け算演算子
	inline SVector operator * (const double _s, const SVector& _v) 
	{
		return { _s * _v.x,_s * _v.y,_s * _v.z };
	}
	
	// 出力ストリーム
	inline std::ostream& operator <<(std::ostream& os, const SVector& v)
	{
		return os << '(' << v.x << ',' << v.y << ',' << v.z << ')';
	}

	//脚座標用のベクトルデータ型．6つの脚それぞれの座標を入れる
	struct SLegVector 
	{
		SVector leg[HexapodConst::LEG_NUM];
	};

	// v1 と v2 が等しいならばtrueを返す．errorは許容誤差．
	bool isEqualVector(const SVector& _v1, const SVector& _v2, const double _error);

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

	// ベクトルの外積 a×b を求める．ベクトルの外積 a×b の向きは，ベクトルaの向きからベクトルbの向きへ右ネジを回転させたとき，ネジの進む向きとする．
	__inline SVector VCross(const SVector &_a, const SVector &_b)
	{
		SVector Result;
		Result.x = _a.y * _b.z - _a.z * _b.y;
		Result.y = _a.z * _b.x - _a.x * _b.z;
		Result.z = _a.x * _b.y - _a.y * _b.x;
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


	// 2次元のベクトルを表す構造体
	struct Vector2
	{
		double x;
		double y;

		Vector2() = default;

		constexpr Vector2(double _x, double _y)
			: x(_x)
			, y(_y) {}

		double length() const { return std::sqrt(lengthSquare()); }
		constexpr double lengthSquare() const { return dot(*this); }
		constexpr double dot(const Vector2& other) const { return x * other.x + y * other.y; }
		double distanceFrom(const Vector2& other) const { return (other - *this).length(); }
		Vector2 normalized() const { return *this / length(); }
		constexpr bool isZero() const { return x == 0.0 && y == 0.0; }

		constexpr Vector2 operator +() const { return *this; }
		constexpr Vector2 operator -() const { return{ -x, -y }; }
		constexpr Vector2 operator +(const Vector2& other) const { return{ x + other.x, y + other.y }; }
		constexpr Vector2 operator -(const Vector2& other) const { return{ x - other.x, y - other.y }; }
		constexpr Vector2 operator *(double s) const { return{ x * s, y * s }; }
		constexpr Vector2 operator /(double s) const { return{ x / s, y / s }; }

		Vector2& operator +=(const Vector2& other)
		{
			x += other.x;
			y += other.y;
			return *this;
		}

		Vector2& operator -=(const Vector2& other)
		{
			x -= other.x;
			y -= other.y;
			return *this;
		}

		Vector2& operator *=(double s)
		{
			x *= s;
			y *= s;
			return *this;
		}

		Vector2& operator /=(double s)
		{
			x /= s;
			y /= s;
			return *this;
		}
	};

	inline constexpr Vector2 operator *(double s, const Vector2& v)
	{
		return{ s * v.x, s * v.y };
	}

	template <class Char>
	inline std::basic_ostream<Char>& operator <<(std::basic_ostream<Char>& os, const Vector2& v)
	{
		return os << Char('(') << v.x << Char(',') << v.y << Char(')');
	}

	template <class Char>
	inline std::basic_istream<Char>& operator >>(std::basic_istream<Char>& is, Vector2& v)
	{
		Char unused;
		return is >> unused >> v.x >> unused >> v.y >> unused;
	}
}
