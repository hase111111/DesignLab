#pragma once
#include <string>

namespace StrtoVal
{
	//! @brief 文字列をint型の変数に変換する
	//! @param [in] _str 変換前の文字列
	//! @return 変換後のint型の変数
	//! @note 変換に失敗した場合は0を返す
	inline int StrToInt(const std::string& _str)
	{
		int res = 0;

		try
		{
			res = std::stoi(_str);
			return res;
		}
		catch (...)
		{
			return 0;
		}
	}
}