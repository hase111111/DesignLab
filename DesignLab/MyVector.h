#pragma once
#include "HexapodConst.h"
#include "MyMath.h"
#include <cmath>
#include <fstream>
#include <iostream>

namespace my_vec 
{
	struct SVector
	{
		SVector() = default;
		constexpr SVector(const float _x, const float _y, const float _z) : x(_x), y(_y), z(_z) {};	

		float x;	//!< ロボットの正面方向に正．
		float y;	//!< ロボットの左向きに正．
		float z;	//!< ロボットの上向きに正．


		//! ベクトルの長さの2乗を返す．sqrt(ルートの計算)がまぁまぁ重いのでこっちを使えるなら使うべき．
		//! @return float x,y,zの値を2乗にして足し合わせたスカラー値
		constexpr float lengthSquare() const { return x * x + y * y + z * z; }	

		//! ベクトルの長さを返す．sqrt(ルートの計算)がまぁまぁ重いので，lengthSquareでいいならこっち使うべき．
		//! @return float ベクトルの長さ
		float length() const { return std::sqrt(lengthSquare()); }

		//! 単位ベクトルを返す．normalizeとは，ベクトルを正規化（単位ベクトルに変換）する操作を表す．
		//! @return SVector 正規化されたベクトル
		SVector normalized() const { return *this / length(); }

		//! x,y,zともに絶対値が許容誤差以下の値ならばtrueを返す．
		//! @return bool 0ならばtrue そうでなければfalse
		bool isZero() const;
		
		 
		//関係演算子、等価演算子
		constexpr bool operator == (const SVector& _v) const 
		{
			if (_v.x - my_math::ALLOWABLE_ERROR < x && x < _v.x + my_math::ALLOWABLE_ERROR)
			{
				if (_v.y - my_math::ALLOWABLE_ERROR < y && y < _v.y + my_math::ALLOWABLE_ERROR)
				{
					if (_v.z - my_math::ALLOWABLE_ERROR < z && z < _v.z + my_math::ALLOWABLE_ERROR) { return true; }
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
		constexpr SVector operator *(const float _s) const { return { x * _s, y * _s, z * _s }; }
		constexpr SVector operator /(const float _s) const { return { x / _s, y / _s, z / _s }; }
		SVector& operator *= (const float _s);
		SVector& operator /= (const float _s);
	};
	
	//! @brief スカラーが先に来る場合の掛け算演算子
	inline SVector operator * (const float _s, const SVector& _v) 
	{
		return { _s * _v.x,_s * _v.y,_s * _v.z };
	}
	
	//! @brief a×b の外積の結果を返す．
	//! @param [in] _a 外積の掛け算：前のベクトル
	//! @param [in] _b 外積の掛け算：後ろのベクトル
	//! @return SVector 外積の結果．ちなみに，a→bへ回転する右ねじが進む方向のベクトルが出力される
	inline SVector getCross(const SVector& _a, const SVector& _b)
	{
		return { _a.y * _b.z - _a.z * _b.y,  _a.z * _b.x - _a.x * _b.z,  _a.x * _b.y - _a.y * _b.x };
	}

	// 出力ストリーム
	template <class Char>
	inline std::basic_ostream<Char>& operator <<(std::basic_ostream<Char>& os, const SVector& v)
	{
		return os << Char('(') << v.x << Char(',') << v.y << Char(',') << v.z << Char(')');
	}

	//入力ストリーム
	template <class Char>
	inline std::basic_istream<Char>& operator >>(std::basic_istream<Char>& is, SVector& v)
	{
		Char unused;
		return is >> unused >> v.x >> unused >> v.y >> unused >> v.z >> unused;
	}


	struct SRotator
	{
		SRotator() = default;
		constexpr SRotator(const float _r, const float _p, const float _y) : roll(_r), pitch(_p), yaw(_y) {};

		float roll, pitch, yaw;

		constexpr SRotator operator *(const float _s) const { return { roll * _s, pitch * _s, yaw * _s }; }
	};

	// v1 と v2 が等しいならばtrueを返す．errorは許容誤差．
	inline bool isEqualVector(const SVector& _v1, const SVector& _v2, const float _error)
	{
		if ((abs(_v1.x - _v2.x) < _error) && (abs(_v1.y - _v2.y) < _error) && (abs(_v1.z - _v2.z) < _error))
		{
			return true;
		}

		return false;
	}

	//! @brief 回転させたベクトルを返す．三角関数の処理が多く重たいので注意．
	//! @param [in] _v 位置ベクトル
	//! @param [in] _rot 回転ベクトル
	//! @return SVector 回転した後の位置ベクトル
	inline SVector rotVector(const SVector& _v, const SRotator& _rot) 
	{
		float _x = cos(_rot.yaw) * cos(_rot.pitch) * _v.x
				+ (cos(_rot.yaw) * sin(_rot.pitch) * sin(_rot.roll) - sin(_rot.yaw) * cos(_rot.roll)) * _v.y
				+ (cos(_rot.yaw) * sin(_rot.pitch) * cos(_rot.roll) + sin(_rot.yaw) * sin(_rot.roll)) * _v.z;

		float _y = sin(_rot.yaw) * cos(_rot.pitch) * _v.x
				+ (sin(_rot.yaw) * sin(_rot.pitch) * sin(_rot.roll) + cos(_rot.yaw) * cos(_rot.roll)) * _v.y
				+ (sin(_rot.yaw) * sin(_rot.pitch) * cos(_rot.roll) - cos(_rot.yaw) * sin(_rot.roll)) * _v.z;

		float _z = -sin(_rot.pitch) * _v.x + cos(_rot.pitch) * sin(_rot.roll) * _v.y + cos(_rot.pitch) * cos(_rot.roll) * _v.z;

		return SVector(_x, _y, _z);
	}

	inline SVector VGet(const float x, const float y, const float z)
	{
		return SVector(x, y, z);
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
	__inline float VDot(const SVector &In1, const SVector &In2)
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

	// ベクトルの大きさ
	__inline float VMag(const SVector &In1) 
	{
		return sqrt(VDot(In1, In1));
	}

	// ベクトルの大きさを１に
	__inline SVector  VUnit(const SVector &In1) 
	{
		return In1 * (1 / VMag(In1));
	}

	// ベクトルの大きさ 位置ベクトルから
	__inline float VMag2(const SVector &In1, const SVector &In2) 
	{
		return VMag(In1 - In2);
	}

	//脚座標用のベクトルデータ型．6つの脚それぞれの座標を入れる
	struct SLegVector
	{
		SVector leg[HexapodConst::LEG_NUM];
	};

	////////////////////////////////////////////////////////////////////////////
	//座標系はmainの座標系でPRYそれぞれに角度を入れたときの回転は
	//VRot(my_vec::VECTOR In, my_vec::VECTOR center, float thP, float thR, float thY)
	//Y(z軸左回り)→P(x軸左回り)→R(y軸左回り)
	//VRot(const my_vec::VECTOR &In, float thP, float thR,float thY )
	//Y(z軸右回り)→P(y軸右回り)→R(x軸右回り)
	//VRot_R(my_vec::VECTOR In, my_vec::VECTOR center, float thP, float thR, float thY)
	//R(y軸左回り)→P(x軸左回り)→Y(z軸左回り)
	///////////////////////////////////////////////////////////////////////////
	//ベクトルの3Dローテーション オイラー角（テイト・ブライアン角）Y-X-Z
	__inline SVector VRot(const SVector In, const SVector center, const float thP, const float thR, const float thY) 
	{
		SVector ans, buf;
		buf = In - center;
		ans.x = (cos(thR) * cos(thY) + sin(thR) * sin(thP) * sin(thY)) * buf.x +
			(cos(thY) * sin(thR) * sin(thP) - cos(thR) * sin(thY)) * buf.y +
			(cos(thP) * sin(thR)) * buf.z;

		ans.y = cos(thP) * sin(thY) * buf.x +
			cos(thP) * cos(thY) * buf.y +
			(-sin(thP))        * buf.z;

		ans.z = (cos(thR) * sin(thP) * sin(thY) - cos(thY) * sin(thR)) * buf.x +
			(cos(thR) * cos(thY) * sin(thP) + sin(thR) * sin(thY)) * buf.y +
			(cos(thR) * cos(thP)) * buf.z;

		return ans + center;
	}

	//上の関数と統合する予定だったけど...
	__inline SVector VRot(const SVector &In, const float thP, const float thR, const float thY)
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
	__inline SVector VRot_R(SVector In, SVector center, const float thP, const float thR, const float thY) 
	{

		SVector ans, buf;
		buf = In - center;
		ans.x = (cos(thY) * cos(thR) - sin(thY) * sin(thP) * sin(thR)) * buf.x +
			(-cos(thP) * sin(thY)) * buf.y +
			(cos(thR) * sin(thY) * sin(thP) + cos(thY) * sin(thR)) * buf.z;

		ans.y = (cos(thY) * sin(thP) * sin(thR) + cos(thR) * sin(thY)) * buf.x +
			(cos(thY) * cos(thP)) * buf.y +
			(sin(thY) * sin(thR) - cos(thY) * cos(thR) * sin(thP)) * buf.z;


		ans.z = -cos(thP) * sin(thR)	* buf.x +
			sin(thP)				* buf.y +
			cos(thP) * cos(thR)	* buf.z;
		return ans + center;
	}

	//2次元用
	// ベクトルの内積
	__inline float V2Dot(const SVector &In1, const SVector &In2)
	{
		return In1.x * In2.x + In1.y * In2.y;
	}

	// 【2次元用】ベクトルの外積を得る関数．Zの値を無視して計算する
	__inline float	getVecCrossXY(const SVector &In1, const SVector &In2)
	{
		return In1.x * In2.y - In1.y * In2.x;
	}

	// ベクトルのサイズの２乗
	__inline float	V2SquareSize(const SVector &In)
	{
		return In.x * In.x + In.y * In.y;
	}

	// ベクトルの大きさ
	__inline float V2Mag(const SVector &In1) 
	{
		return sqrt(V2Dot(In1, In1));
	}

	// ベクトルの大きさを１に
	__inline SVector  V2Unit(const SVector &In1) 
	{
		return In1 * (1 / V2Mag(In1));
	}

	// ベクトルの大きさ 位置ベクトルから
	__inline float V2Mag2(const SVector &In1, const SVector &In2) 
	{
		return V2Mag(In1 - In2);
	}


	// 2次元のベクトルを表す構造体
	struct SVector2
	{
		float x;
		float y;

		SVector2() = default;

		constexpr SVector2(float _x, float _y)
			: x(_x)
			, y(_y) {}

		float length() const { return std::sqrt(lengthSquare()); }
		constexpr float lengthSquare() const { return dot(*this); }
		constexpr float dot(const SVector2& other) const { return x * other.x + y * other.y; }
		float distanceFrom(const SVector2& other) const { return (other - *this).length(); }
		SVector2 normalized() const { return *this / length(); }
		constexpr bool isZero() const { return x == 0.0 && y == 0.0; }

		constexpr SVector2 operator +() const { return *this; }
		constexpr SVector2 operator -() const { return{ -x, -y }; }
		constexpr SVector2 operator +(const SVector2& other) const { return{ x + other.x, y + other.y }; }
		constexpr SVector2 operator -(const SVector2& other) const { return{ x - other.x, y - other.y }; }
		constexpr SVector2 operator *(float s) const { return{ x * s, y * s }; }
		constexpr SVector2 operator /(float s) const { return{ x / s, y / s }; }

		SVector2& operator +=(const SVector2& other)
		{
			x += other.x;
			y += other.y;
			return *this;
		}

		SVector2& operator -=(const SVector2& other)
		{
			x -= other.x;
			y -= other.y;
			return *this;
		}

		SVector2& operator *=(float s)
		{
			x *= s;
			y *= s;
			return *this;
		}

		SVector2& operator /=(float s)
		{
			x /= s;
			y /= s;
			return *this;
		}
	};

	inline constexpr SVector2 operator *(float s, const SVector2& v)
	{
		return{ s * v.x, s * v.y };
	}

	template <class Char>
	inline std::basic_ostream<Char>& operator <<(std::basic_ostream<Char>& os, const SVector2& v)
	{
		return os << Char('(') << v.x << Char(',') << v.y << Char(')');
	}

	template <class Char>
	inline std::basic_istream<Char>& operator >>(std::basic_istream<Char>& is, SVector2& v)
	{
		Char unused;
		return is >> unused >> v.x >> unused >> v.y >> unused;
	}
}

//! @file vectorFunc.h
//! @brief 位置ベクトルを表現する構造体の実装
//! @author 長谷川

//! @namespace my_vec
//! @brief 位置ベクトルを表現する構造体SVectorや回転を表現するSRotatorが実装されている．
//! @details Vectorというクラス，構造体は既に2つもある．1つはDxlibのVECTOR，もう1つは言わずとしれたstd::vector．<br>
//! 両方とも本プロジェクトから切り離しようのないものであるので名前がぶつかることのないように命名する必要がある．<br>
//! そのため先頭にStructを意味するSをつけてある．<br> 
//! また名前空間に入れることで万が一の時の名前の衝突を防いでいる．<br>
//! そのため，using namespace my_vec; はあまり書かないように! どうしても書くならば，cppファイル内に書くこと．

//! @struct SVector 
//! @brief ベクトルを表す構造体
//! @details 座標系はロボットの進行方向にXの正，ロボットの上向きにZの正，右手座標系にYをとっている<br> 
//! <br>
//!	ヘッダファイル内に実装を書くのは個人的には避けたいのだが，constexpr関数を使う場合，このようにする必要があるらしい．<br>
//!	実行速度が大切なプロジェクトであるため，このように処理を記述する．<br> 
//! <br>
//! [参考資料] <br> 
//! https://qiita.com/Reputeless/items/96226cfe1282a014b147 <br> 
//! https://qiita.com/KRiver1/items/ef7731467b5ca83850cb <br>
//!	https://atcoder.jp/contests/apg4b/tasks/APG4b_ab?lang=ja ←細かい話」のコンストラクタの項 <br>
//!	https://programming.pc-note.net/cpp/operator2.html ←比較演算子 <br>
//!	http://marupeke296.com/COL_main.html ←先輩がおそらく衝突判定の参考資料にしたであろうサイト <br>
//! @author 長谷川

//! @struct SRotator
//! @brief 回転を表す構造体．XYZオイラー角
//! @details XYZオイラー角によって回転を表現する．<br>
//! ロール(X軸)，ピッチ(Y軸)，ヨー(Z軸)はそれぞれ右ねじの方向に回転する．<br>
//! 参考資料 :  https://watako-lab.com/2019/01/23/roll_pitch_yaw/
//! 
