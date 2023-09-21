//! @file designlab_vector3.h
//! @brief 位置ベクトルを表現する構造体

#ifndef DESIGNLAB_VECTOR3_H_
#define DESIGNLAB_VECTOR3_H_


#include <cmath>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <ios>

#include "designlab_math_util.h"
#include "designlab_vector2.h"
#include "hexapod_const.h"


namespace designlab
{

	//! @struct designlab::Vector3
	//! @brief 3次元の位置ベクトルを表す構造体
	//! @details 座標系はロボットの進行方向にXの正，ロボットの上向きにZの正，右手座標系にYをとっている 
	//! @n
	//!	@n ヘッダファイル内に実装を書くのは個人的には避けたいのだが，constexpr関数を使う場合，このようにする必要があるらしい．
	//!	@n 実行速度が大切なプロジェクトであるため，このように処理を記述する．
	//! @n
	//! @n [参考資料]  
	//! @n https://qiita.com/Reputeless/items/96226cfe1282a014b147  
	//! @n https://qiita.com/KRiver1/items/ef7731467b5ca83850cb 
	//!	@n https://atcoder.jp/contests/apg4b/tasks/APG4b_ab?lang=ja ←細かい話」のコンストラクタの項
	//!	@n https://programming.pc-note.net/cpp/operator2.html ←比較演算子 
	//!	@n http://marupeke296.com/COL_main.html ←先輩がおそらく衝突判定の参考資料にしたであろうサイト
	//! @n 
	//! @n Vectorというクラス，構造体は既に2つもある．1つはDxlibのVECTOR，もう1つは言わずとしれたstd::vector．
	//! @n 両方とも本プロジェクトから切り離しようのないものであるので名前がぶつかることのないように命名する必要がある．	
	//! @n 名前空間に入れることで万が一の時の名前の衝突を防いでいる．
	//! @n そのため，using namespace designlab; はあまり書かないように! 
	//! @n どうしても書くならば，cppファイル内に using namespace dl = designlab; と書いて，dl::Vector3 と書くようにする．
	struct Vector3 final
	{
		constexpr Vector3() : x(0.f), y(0.f), z(0.f) {};
		constexpr Vector3(const float x_pos, const float y_pos, const float z_pos) : x(x_pos), y(y_pos), z(z_pos) {};

		constexpr Vector3(const Vector3& other) = default;					//コピーコンストラクタ
		constexpr Vector3(Vector3&& other) noexcept = default;				//ムーブコンストラクタ
		constexpr Vector3& operator = (const Vector3& other) = default;		//コピー代入演算子

		~Vector3() = default;


		//関係演算子、等価演算子
		constexpr bool operator == (const Vector3& v) const
		{
			if (designlab::math_util::IsEqual(v.x, x) && designlab::math_util::IsEqual(v.y, y) && designlab::math_util::IsEqual(v.z, z))
			{
				return true;
			}

			return false;
		}
		constexpr bool operator != (const Vector3& other) const { return !(*this == other); }
		bool operator < (const Vector3& other) const { return Length() < other.Length(); }
		bool operator > (const Vector3& other) const { return other < *this; }
		bool operator <= (const Vector3& other) const { return !(*this > other); }
		bool operator >= (const Vector3& other) const { return !(*this < other); }

		//算術演算子
		constexpr Vector3 operator +() const { return *this; }
		constexpr Vector3 operator -() const { return { -x, -y, -z }; }
		constexpr Vector3 operator +(const Vector3& other) const { return { x + other.x , y + other.y, z + other.z }; }
		constexpr Vector3 operator -(const Vector3& other) const { return { x - other.x , y - other.y, z - other.z }; }
		Vector3& operator += (const Vector3& other);
		Vector3& operator -= (const Vector3& other);
		constexpr Vector3 operator *(const float num) const { return { x * num, y * num, z * num }; }
		constexpr Vector3 operator /(const float num) const { return { x / num, y / num, z / num }; }
		Vector3& operator *= (const float num);
		Vector3& operator /= (const float num);


		//! @brief ベクトルの長さの2乗を返す．sqrt(ルートの計算)がまぁまぁ重いのでこっちを使えるなら使うべき．
		//! @return float x,y,zの値を2乗にして足し合わせたスカラー値
		constexpr float LengthSquare() const { return x * x + y * y + z * z; }

		//! @brief ベクトルの長さを返す．sqrt(ルートの計算)がまぁまぁ重いので，lengthSquareでいいならそっちを使うべき．
		//! @return float ベクトルの長さ
		inline float Length() const { return std::sqrt(LengthSquare()); }

		//! @brief 単位ベクトルを返す．normalizeとは，ベクトルを正規化（単位ベクトルに変換）する操作を表す．絶対値が0のベクトルを使用しないこと!
		//! @return Vector3 正規化されたベクトル
		Vector3 Normalized() const;

		//! @brief x,y,zともに絶対値が許容誤差以下の値ならばtrueを返す．
		//! @return bool 0ならばtrue そうでなければfalse
		bool IsZero() const;

		//! @brief 別のベクトルとこのベクトルの距離を返す．
		//! @param [in] other 別のベクトル
		//! @return float 距離
		inline float DistanceFrom(const Vector3& other) const { return (*this - other).Length(); }

		//! @brief 自分・引数 の内積の結果を返す．
		//! @param [in] other もう一方のベクトル
		//! @return float 内積の結果
		constexpr float Dot(const Vector3& other) const { return x * other.x + y * other.y + z * other.z; }

		//! @brief 自分×引数 の外積の結果を返す．
		//! @param [in] other 外積の掛け算：後ろのベクトル
		//! @return Vector3 外積の結果．ちなみに，自分→引数へ回転する右ねじが進む方向のベクトルが出力される
		constexpr Vector3 Cross(const Vector3& other) const
		{
			return { y * other.z - z * other.y,  z * other.x - x * other.z,  x * other.y - y * other.x };
		}

		//! @brief XY平面に射影したベクトルを返す．
		//! @return Vector2 XY平面に射影したベクトル
		constexpr Vector2 ProjectedXY() const { return { x, y }; }

		//! @brief 正面に進む単位ベクトルを返す
		//! @return Vector3 正面方向の単位ベクトル，xの正方向
		constexpr static Vector3 GetFrontVec() { return {1,0,0}; }

		//! @brief 左に進む単位ベクトルを返す
		//! @return Vector3 左方向の単位ベクトル，xの正方向
		constexpr static Vector3 GetLeftVec() { return { 0,1,0 }; }


		//! @brief このベクトルを文字列にして返す
		//! @n (x, y) の形式，小数点以下3桁まで
		//! @return このベクトルを文字列にしたもの
		std::string ToString() const;


		float x;	//!< ロボットの正面方向に正．
		float y;	//!< ロボットの左向きに正．
		float z;	//!< ロボットの上向きに正．
	};


	//! @brief スカラーが先に来る場合の掛け算演算子
	constexpr Vector3 operator * (const float s, const Vector3& vec)
	{
		return { s * vec.x, s * vec.y, s * vec.z };
	}


	// 出力ストリーム
	template <class Char>
	std::basic_ostream<Char>& operator <<(std::basic_ostream<Char>& os, const Vector3& v)
	{
		return os << ::designlab::math_util::ConvertFloatToString(v.x) << Char(',') << 
			::designlab::math_util::ConvertFloatToString(v.y) << Char(',') <<
			::designlab::math_util::ConvertFloatToString(v.z);
	}


	//入力ストリーム
	template <class Char>
	inline std::basic_istream<Char>& operator >>(std::basic_istream<Char>& is, Vector3& v)
	{
		Char unused;
		return is >> unused >> v.x >> unused >> v.y >> unused >> v.z >> unused;
	}

} // namespace designlab


#endif // !DESIGNLAB_VECTOR3_H_