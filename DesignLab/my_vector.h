#pragma once

#include <cmath>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <ios>

#include "HexapodConst.h"
#include "my_math.h"
#include "my_vector2.h"


//! @namespace my_vec
//! @date 2023/08/06
//! @auther 長谷川
//! @brief 位置ベクトルを表現する構造体SVectorや回転を表現するSRotatorが実装されている．
//! @details Vectorというクラス，構造体は既に2つもある．1つはDxlibのVECTOR，もう1つは言わずとしれたstd::vector．
//! @n 両方とも本プロジェクトから切り離しようのないものであるので名前がぶつかることのないように命名する必要がある．
//! @n そのため先頭にStructを意味するSをつけてある．
//! @n また名前空間に入れることで万が一の時の名前の衝突を防いでいる．
//! @n そのため，using namespace my_vec; はあまり書かないように! どうしても書くならば，cppファイル内に書くこと．
namespace my_vec
{

	//! @struct my_vec::SVector
	//! @date 2023/08/06 
	//! @author 長谷川
	//! @brief ベクトルを表す構造体
	//! @details 座標系はロボットの進行方向にXの正，ロボットの上向きにZの正，右手座標系にYをとっている @n
	//!	@n ヘッダファイル内に実装を書くのは個人的には避けたいのだが，constexpr関数を使う場合，このようにする必要があるらしい．
	//!	@n 実行速度が大切なプロジェクトであるため，このように処理を記述する．@n
	//! @n [参考資料]  
	//! @n https://qiita.com/Reputeless/items/96226cfe1282a014b147  
	//! @n https://qiita.com/KRiver1/items/ef7731467b5ca83850cb 
	//!	@n https://atcoder.jp/contests/apg4b/tasks/APG4b_ab?lang=ja ←細かい話」のコンストラクタの項
	//!	@n https://programming.pc-note.net/cpp/operator2.html ←比較演算子 
	//!	@n http://marupeke296.com/COL_main.html ←先輩がおそらく衝突判定の参考資料にしたであろうサイト 
	struct SVector final
	{
		SVector() = default;
		constexpr SVector(const float x_pos, const float y_pos, const float z_pos) : x(x_pos), y(y_pos), z(z_pos) {};


		//関係演算子、等価演算子
		constexpr bool operator == (const SVector& v) const
		{
			if (my_math::isEqual(v.x, x) && my_math::isEqual(v.y, y) && my_math::isEqual(v.z, z))
			{
				return true;
			}
			return false;
		}
		constexpr bool operator != (const SVector& other) const { return !(*this == other); }
		bool operator < (const SVector& other) const { return length() < other.length(); }
		bool operator > (const SVector& other) const { return other < *this; }
		bool operator <= (const SVector& other) const { return !(*this > other); }
		bool operator >= (const SVector& other) const { return !(*this < other); }

		//算術演算子
		constexpr SVector operator +() const { return *this; }
		constexpr SVector operator -() const { return { -x, -y, -z }; }
		constexpr SVector operator +(const SVector& other) const { return { x + other.x , y + other.y, z + other.z }; }
		constexpr SVector operator -(const SVector& other) const { return { x - other.x , y - other.y, z - other.z }; }
		SVector& operator += (const SVector& other);
		SVector& operator -= (const SVector& other);
		constexpr SVector operator *(const float _s) const { return { x * _s, y * _s, z * _s }; }
		constexpr SVector operator /(const float _s) const { return { x / _s, y / _s, z / _s }; }
		SVector& operator *= (const float _s);
		SVector& operator /= (const float _s);


		//! @brief ベクトルの長さの2乗を返す．sqrt(ルートの計算)がまぁまぁ重いのでこっちを使えるなら使うべき．
		//! @return float x,y,zの値を2乗にして足し合わせたスカラー値
		constexpr float lengthSquare() const { return x * x + y * y + z * z; }

		//! @brief ベクトルの長さを返す．sqrt(ルートの計算)がまぁまぁ重いので，lengthSquareでいいならこっち使うべき．
		//! @return float ベクトルの長さ
		float length() const { return std::sqrt(lengthSquare()); }

		//! @brief 単位ベクトルを返す．normalizeとは，ベクトルを正規化（単位ベクトルに変換）する操作を表す．絶対値が0のベクトルを使用しないこと!
		//! @return SVector 正規化されたベクトル
		SVector normalized() const { return *this / length(); }

		//! @brief x,y,zともに絶対値が許容誤差以下の値ならばtrueを返す．
		//! @return bool 0ならばtrue そうでなければfalse
		bool isZero() const;

		//! @brief 別のベクトルとこのベクトルの距離を返す．
		//! @param [in] other 別のベクトル
		//! @return float 距離
		float distanceFrom(const SVector& other) const { return (*this - other).length(); }

		//! @brief 自分・引数 の内積の結果を返す．
		//! @param [in] other もう一方のベクトル
		//! @return float 内積の結果
		constexpr float dot(const SVector& other) const { return x * other.x + y * other.y + z * other.z; }

		//! @brief 自分×引数 の外積の結果を返す．
		//! @param [in] other 外積の掛け算：後ろのベクトル
		//! @return SVector 外積の結果．ちなみに，自分→引数へ回転する右ねじが進む方向のベクトルが出力される
		constexpr SVector cross(const SVector& other) const
		{
			return { y * other.z - z * other.y,  z * other.x - x * other.z,  x * other.y - y * other.x };
		}

		//! @brief XY平面に射影したベクトルを返す．
		//! @return SVector2 XY平面に射影したベクトル
		constexpr SVector2 projectedXY() const { return { x, y }; }


		float x;	//!< ロボットの正面方向に正．
		float y;	//!< ロボットの左向きに正．
		float z;	//!< ロボットの上向きに正．
	};


	//! @brief スカラーが先に来る場合の掛け算演算子
	constexpr SVector operator * (const float s, const SVector& vec)
	{
		return { s * vec.x, s * vec.y, s * vec.z };
	}


	// 出力ストリーム
	inline std::ostream& operator<<(std::ostream& os, const SVector& v)
	{
		const int width = 10;
		const char fill = ' ';
		const int precision = 3;

		os << std::fixed << std::setprecision(precision);
		os << "(x : ";
		os << std::setw(width) << std::setfill(fill) << v.x;
		os << ", y : ";
		os << std::setw(width) << std::setfill(fill) << v.y;
		os << ", z : ";
		os << std::setw(width) << std::setfill(fill) << v.z;
		os << ")";
		return os;
	}


	//入力ストリーム
	template <class Char>
	inline std::basic_istream<Char>& operator >>(std::basic_istream<Char>& is, SVector& v)
	{
		Char unused;
		return is >> unused >> v.x >> unused >> v.y >> unused >> v.z >> unused;
	}

} // namespace my_vec


//! @file my_vector.h
//! @date 2023/08/06
//! @author 長谷川
//! @brief 位置ベクトルを表現する構造体
//! @n 行数 : @lineinfo
