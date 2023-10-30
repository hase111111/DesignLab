//! @file designlab_string_util.h
//! @brief ������������֐����܂Ƃ߂����O��ԁD


#ifndef	DESIGNLAB_STRING_UTIL_H_
#define	DESIGNLAB_STRING_UTIL_H_


#include <string>
#include <vector>


namespace designlab 
{
	namespace string_util
	{
		//! @brief ������𕪊�����֐��D
		//! @param [in] str �������镶����D
		//! @param [in] delim �������镶���D
		//! @return std::vector<std::string> ��������������̔z��D
		std::vector<std::string> Split(const std::string& str, const char delim);

		//! @brief ������𕪊�����֐��D
		//! @param [in] str �������镶����D
		//! @param [in] delim �������镶���D2�����ȏ�̕�������w��ł��Ȃ��D
		//! @return std::vector<std::string> ��������������D
		std::vector<std::string> Split(const std::string& str, const std::string& delim);

	}	// namespace string_util

}	// namespace designlab


#endif	