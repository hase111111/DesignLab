//! @file designlab_array_util.h
//! @brief std::array���쐬����֐��D


#ifndef DESIGNLAB_ARRAY_UTIL_H_
#define DESIGNLAB_ARRAY_UTIL_H_


#include <array>


namespace designlab 
{
	//! @brief std::array���쐬����֐��D
	//! @n ���̊֐����쐬�������`�x�[�V�����Ƃ��ẮCstd::array��constexpr�ŏ���������ۂɋ�킵�����߁D
	//! @n ���̊֐����g�����ƂŁCstd::array��constexpr�ŏ��������邱�Ƃ��ł���D
	//! @n �Ⴆ�΁Cstd::array<int, 3>���쐬����ꍇ�́Cmake_array<int>(1, 2, 3)�Ƃ���D
	template<typename T, typename ...Args>
	constexpr std::array<T, sizeof...(Args)> make_array(Args&&... args)
	{
		return std::array<T, sizeof...(Args)>{ static_cast<Args&&>(args)... };
	}

}  // namespace designlab


#endif