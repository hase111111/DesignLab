#pragma once

#include <string>


namespace StrtoVal
{
	//! @brief �������int�^�̕ϐ��ɕϊ�����
	//! @param [in] _str �ϊ��O�̕�����
	//! @return �ϊ����int�^�̕ϐ�
	//! @note �ϊ��Ɏ��s�����ꍇ��0��Ԃ�
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