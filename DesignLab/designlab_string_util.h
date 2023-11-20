//! @file designlab_string_util.h
//! @brief ������������֐����܂Ƃ߂����O��ԁD


#ifndef	DESIGNLAB_STRING_UTIL_H_
#define	DESIGNLAB_STRING_UTIL_H_


#include <string>
#include <vector>

#include <magic_enum.hpp>


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

		//! @brief enum�𕶎���ɕϊ�����֐��D
		//! @n Google C++ coding style ����enum�̗v�f�� �擪��k�����ăL�������P�[�X�ŏ������Ƃ���������Ă���D
		//! @n �Ⴆ�� enum class Color { kRed, kGreen, kBlue } �Ə����D
		//! @n ���̂��߁C���̊֐��͂���k����������@�\��񋟂��CColor::kRed ��n���� "Red" �Ƃ����������Ԃ��D
		//! @param [in] enum_value enum�̗v�f�D
		//! @return std::string enum�̗v�f�𕶎���ɂ������́D
		//! @tparam T enum�^�D
		template <typename T>
		std::string MyEnumToString( const T& enum_value )
		{
			//�^�`�F�b�N T��enum�^�ł��邱�Ƃ��`�F�b�N����D
			static_assert( std::is_enum<T>::value, "������enum�C���邢��enum class�ł���K�v������܂��D" );

			std::string str = static_cast<std::string>(magic_enum::enum_name(enum_value));

			if ( str.size() > 0 && str[0] == 'k' )
			{
				// �擪��k���폜����D
				str.erase( 0, 1 );
			}

			return str;
		}

	}	// namespace string_util

}	// namespace designlab


#endif	// DESIGNLAB_STRING_UTIL_H_
