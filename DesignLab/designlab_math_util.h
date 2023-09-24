//! @file designlab_math_util.h
//! @brief 基本的な計算を行う関数．


#ifndef DESIGNLAB_MATH_UTIL_H_
#define DESIGNLAB_MATH_UTIL_H_


#include <string>
#include <vector>


namespace designlab 
{
	//! @namespacse math_util 基本的な計算を行う関数をまとめた名前空間．
	//! @brief 基本的な計算を行う関数をまとめた名前空間．
	namespace math_util 
	{
		constexpr double kDoublePi = 3.141592653589793;		//!< double型の円周率
		constexpr float kFloatPi = 3.141592653589793f;		//!< float型の円周率

		constexpr double kDoubleAllowableError = 0.001;								//!< これ以上小さい値は0とみなす．allowable error，許容誤差のこと．0.1とか，大きな値は非推奨．
		constexpr float kAllowableError = static_cast<float>(kDoubleAllowableError);	//!< これ以上小さい値は0とみなす．allowable error，許容誤差のこと．0.1とか，大きな値は非推奨．


		//! @brief C++において，小数同士の計算は誤差が出てしまう．誤差込みで値が等しいか調べる．
		//! @param [in] num1 比較する数字1つ目 
		//! @param [in] num2 比較する数字2つ目
		//! @return bool 等しいならばtrue 
		constexpr bool IsEqual(const float num1, const float num2)
		{
			const float dif = num1 - num2;
			if (dif > 0) { return (dif <= kAllowableError); }
			else { return (dif >= -kAllowableError); }
		}

		//! @brief C++において，小数同士の計算は誤差が出てしまう．誤差込みで値が等しいか調べる．
		//! @param [in] num1 比較する数字1つ目 
		//! @param [in] num2 比較する数字2つ目
		//! @return bool 等しいならばtrue 
		constexpr bool IsEqual(const double num1, const double num2)
		{
			const double dif = num1 - num2;
			if (dif > 0) { return (dif <= kDoubleAllowableError); }
			else { return (dif >= -kDoubleAllowableError); }
		}

		//! @brief 2乗した値を返す関数．
		//! @param [in] num 2乗する数．
		//! @return double 2乗した値． 
		constexpr double Squared(const double num) { return num * num; }

		//! @brief 2乗した値を返す関数．
		//! @param [in] num 2乗する数．
		//! @return float 2乗した値． 
		constexpr float Squared(const float num) { return num * num; }

		//! @brief 2乗した値を返す関数．
		//! @param [in] num 2乗する数．
		//! @return int 2乗した値． 
		constexpr int Squared(const int num) { return num * num; }


		//! @brief 指定した範囲内の乱数を生成する．
		//! @param [in] min 乱数の最小値．
		//! @param [in] max 乱数の最大値．
		//! @return double 生成した乱数．
		double GenerateRandomNumber(double min, double max);

		//! @brief 指定した範囲内の乱数を生成する．
		//! @param [in] min 乱数の最小値．
		//! @param [in] max 乱数の最大値．
		//! @return float 生成した乱数．
		float GenerateRandomNumber(float min, float max);

		//! @brief 指定した範囲内の乱数を生成する．
		//! @param [in] min 乱数の最小値．
		//! @param [in] max 乱数の最大値．
		//! @return int 生成した乱数．
		int GenerateRandomNumber(int min, int max);


		//! @brief 角度をradからdegに変換する関数．
		//! @param [in] rad 角度[rad]．
		//! @return double 角度[deg]．
		constexpr double ConvertRadToDeg(const double rad) { return rad * 180.0 / kDoublePi; };

		//! @brief 角度をradからdegに変換する関数．
		//! @param [in] rad 角度[rad]．
		//! @return float 角度[deg]．
		constexpr float ConvertRadToDeg(const float rad) { return rad * 180.0f / kFloatPi; };

		//! @brief 角度をdegからradに変換する関数．
		//! @param [in] deg 角度[deg]．
		//! @return double 角度[rad]．
		constexpr double ConvertDegToRad(const double deg) { return deg * kDoublePi / 180.0; }

		//! @brief 角度をdegからradに変換する関数．
		//! @param [in] deg 角度[deg]．
		//! @return float 角度[rad]．
		constexpr float ConvertDegToRad(const float deg) { return deg * kFloatPi / 180.0f; }


		float limitRangeAngle(const float angle);


		constexpr int kDigit = 3;	//!< 小数点以下の桁数．
		constexpr int kWidth = 10;	//!< 文字列の幅．

		//! @brief 小数を文字列に変換する関数．
		//! @n C++ では C のフォーマットのように %3.3f とかで小数を文字列に変換できないため自作する
		//! @param [in] num 変換する小数．
		//! @param [in] digit 小数点以下の桁数．
		//! @param [in] width 文字列の幅．
		//! @return std::string 変換した文字列．
		std::string ConvertFloatToString(const float num, const int digit = kDigit, const int width = kWidth);

		//! @brief 小数を文字列に変換する関数．
		//! @n C++ では C のフォーマットのように %3.3f とかで小数を文字列に変換できないため自作する
		//! @param [in] num 変換する小数．
		//! @param [in] digit 小数点以下の桁数．
		//! @param [in] width 文字列の幅．
		//! @return std::string 変換した文字列．
		std::string ConvertDoubleToString(const double num, const int digit = kDigit, const int width = kWidth);



	}	// namespace math_util

}	// namespace designlab


#endif	// DESIGNLAB_MATH_UTIL_H_