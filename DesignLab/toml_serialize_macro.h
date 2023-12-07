﻿//! @file toml_serialize_macro.h
//! @brief tomlファイルのシリアライズ/デシリアライズを行うためのマクロ．
//! @details TOML11_DEFINE_CONVERSION_NON_INTRUSIVEをラッパしたもの．
//! @n もともとのほうではenum型を取り扱うことができなかったが，このマクロでは取り扱うことができる．
//! @n また，クラスの説明を追加することができる．
//! @n 以下のように使用する．

#ifndef DESIGNLAB_TOML_SERIALIZE_MACRO_H_
#define DESIGNLAB_TOML_SERIALIZE_MACRO_H_


#include <iostream>
#include <map>
#include <string>
#include <vector>

#include <magic_enum.hpp>
#include <strconv2.h>

#include "toml11_define.h"


namespace designlab
{
	//! @namespace toml_func
	//! @brief tomlファイルのシリアライズ/デシリアライズを行うための関数群．
	//! @n 他のファイルから呼び出すことを想定していないので，このように奥まった名前空間に配置している．
	namespace toml_func
	{
		struct Toml11Description final
		{
			//! テーブルがない場合に指定する文字列
			static const std::string NO_TABLE;

			Toml11Description(const std::string& t, const std::string& d) : table_name(t), description(d) {}

			std::string table_name;
			std::string description;
		};

		std::vector<std::string> sjis_to_utf8_vec(const std::vector<std::string>& str_vec);

		//! @brief tomlファイルに値を追加するための関数．
		//! @n enumに関して特殊化されており，enum型の値を文字列に変換してから追加する．
		template <typename T>
		typename std::enable_if<!std::is_enum<T>::value>::type
			SetTomlValue(::toml::basic_value<toml::preserve_comments, std::map>& v, const std::string& str, const T& value)
		{
			v[str] = value;
		}

		template <typename T>
		typename std::enable_if<std::is_enum<T>::value>::type
			SetTomlValue(::toml::basic_value<toml::preserve_comments, std::map>& v, const std::string& str, const T& value)
		{
			v[str] = static_cast<std::string>(magic_enum::enum_name(value));
		}

	} // namespace toml_func

} // namespace designlab


//! @def DESIGNLAB_SUB_MACRO_FIND_MEMBER_VARIABLE_FROM_VALUE
//! @brief DESIGNLAB_DEFINE_CONVERSION_NON_INTRUSIVEの補助マクロ．他のファイルから呼び出さないこと．
//! @n tomlファイルからクラスのメンバ変数を取得する．
#define DESIGNLAB_SUB_MACRO_FIND_MEMBER_VARIABLE_FROM_VALUE(VAR_NAME)                               \
if constexpr (std::is_enum<decltype(obj.VAR_NAME)>::value)                                          \
{                                                                                                   \
    const std::string table_str = desc.VAR_NAME.table_name;                                         \
                                                                                                    \
    if(table_str == ::designlab::toml_func::Toml11Description::NO_TABLE)                            \
    {                                                                                               \
        std::string str = toml::find<std::string>(v, TOML11_STRINGIZE(VAR_NAME));                   \
        obj.VAR_NAME = magic_enum::enum_cast<decltype(obj.VAR_NAME)>(str).value();                  \
    }                                                                                               \
	else                                                                                            \
	{                                                                                               \
		std::string str = toml::find<std::string>(v[table_str], TOML11_STRINGIZE(VAR_NAME));        \
		obj.VAR_NAME = magic_enum::enum_cast<decltype(obj.VAR_NAME)>(str).value();                  \
	}                                                                                               \
}                                                                                                   \
else                                                                                                \
{                                                                                                   \
    const std::string table_str = desc.VAR_NAME.table_name;                                         \
																								    \
	if(table_str == ::designlab::toml_func::Toml11Description::NO_TABLE)                            \
    {																						        \
        obj.VAR_NAME = toml::find<decltype(obj.VAR_NAME)>(v, TOML11_STRINGIZE(VAR_NAME));           \
	}                                                                                               \
	else                                                                                            \
	{                                                                                               \
		obj.VAR_NAME = toml::find<decltype(obj.VAR_NAME)>(v[table_str], TOML11_STRINGIZE(VAR_NAME));\
	}                                                                                               \
}


//! @def DESIGNLAB_SUB_MACRO_ASSIGN_MEMBER_VARIABLE_TO_VALUE
//! @brief DESIGNLAB_DEFINE_CONVERSION_NON_INTRUSIVEの補助マクロ．他のファイルから呼び出さないこと．
//! @n クラスのメンバ変数をtomlファイルに追加する．
#define DESIGNLAB_SUB_MACRO_ASSIGN_MEMBER_VARIABLE_TO_VALUE(VAR_NAME)                                           \
if(desc.VAR_NAME.table_name != ::designlab::toml_func::Toml11Description::NO_TABLE)                             \
{                                                                                                               \
    if(v.count(desc.VAR_NAME.table_name) == 0)                                                                  \
    {													                                                        \
		v[desc.VAR_NAME.table_name] = toml::table{};                                                            \
	}                                                                                                           \
																						                        \
   	::designlab::toml_func::SetTomlValue(v[desc.VAR_NAME.table_name], TOML11_STRINGIZE(VAR_NAME), obj.VAR_NAME);\
}                                                                                                               \
else                                                                                                            \
{                                                                                                               \
    ::designlab::toml_func::SetTomlValue(v, TOML11_STRINGIZE(VAR_NAME), obj.VAR_NAME);                          \
}


//! @def DESIGNLAB_SUB_MACRO_ADD_COMMENT
//! @brief DESIGNLAB_DEFINE_CONVERSION_NON_INTRUSIVEの補助マクロ．他のファイルから呼び出さないこと．
//! @n tomlファイルの要素にクラスの説明を追加する．もし，説明がなければ説明を追加しない．
//! @param VAR_NAME 変数名．
#define DESIGNLAB_SUB_MACRO_ADD_COMMENT(VAR_NAME)                                       \
if (desc.VAR_NAME.description != "")                                                    \
{                                                                                       \
    if(desc.VAR_NAME.table_name != ::designlab::toml_func::Toml11Description::NO_TABLE) \
    {                                                                                   \
    	v[desc.VAR_NAME.table_name][#VAR_NAME].comments().                              \
            push_back(desc.VAR_NAME.description);                                       \
    }                                                                                   \
    else                                                                                \
    {                                                                                   \
    	v[#VAR_NAME].comments().push_back(desc.VAR_NAME.description);                   \
    }                                                                                   \
}


#define DESIGNLAB_TOML11_DESCRIPTION_CLASS(CLASS) \
struct CLASS##Description final


//! @def DESIGNLAB_TOML11_NO_FILE_DESCRIPTION
//! @brief tomlファイルにファイルの説明を追加しないことを示す文字列．
//! @n DESIGNLAB_TOML11_DESCRIPTION_CLASS内に必ず記述する必要がある．
#define DESIGNLAB_TOML11_NO_FILE_DESCRIPTION()				\
const ::std::vector<::std::string> file_description_vec{};

//! @def DESIGNLAB_TOML11_FILE_DESCRIPTION
//! @brief tomlファイルにファイルの説明を追加するためのマクロ．
//! @n DESIGNLAB_TOML11_DESCRIPTION_CLASS内に必ず記述する必要がある．
//! @param DESCRIPTION 説明．文字列で指定する．
#define DESIGNLAB_TOML11_FILE_DESCRIPTION(DESCRIPTION)						\
const ::std::vector<::std::string> file_description_vec{sjis_to_utf8(DESCRIPTION)};

//! @def DESIGNLAB_TOML11_FILE_DESCRIPTION_MULTI_LINE
//! @brief tomlファイルにファイルの説明を追加するためのマクロ．
//! @n DESIGNLAB_TOML11_DESCRIPTION_CLASS内に必ず記述する必要がある．
//! @param DESCRIPTION_VEC 説明．文字列のvectorで指定する．
#define DESIGNLAB_TOML11_FILE_DESCRIPTION_MULTI_LINE(DESCRIPTION_VEC)		\
const ::std::vector<::std::string> file_description_vec = ::designlab::toml_func::sjis_to_utf8_vec(DESCRIPTION_VEC);


//! @def DESIGNLAB_TOML11_ADD_TABLE_DESCRIPTION
//! @brief tomlファイルにテーブルの説明を追加するためのマクロ．
//! @n DESIGNLAB_TOML11_DESCRIPTION_CLASS内に必ず記述する必要がある．
//! @param ... テーブル名と説明．
#define DESIGNLAB_TOML11_ADD_TABLE_DESCRIPTION(...)							\
const std::vector<std::string> table_name_description_vec = ::designlab::toml_func::sjis_to_utf8_vec({__VA_ARGS__});

//! @def DESIGNLAB_TOML11_NO_TABLE
//! @brief tomlファイルに追加するテーブルにコメントを追加するためのマクロ．
//! @n DESIGNLAB_TOML11_DESCRIPTION_CLASS内に必ず記述する必要がある．
#define DESIGNLAB_TOML11_NO_TABLE_DESCRIPTION()			\
const std::vector<std::string> table_name_description_vec = {};


//! @def DESIGNLAB_ADD_FILE_DESCRIPTION
//! @brief tomlファイルにファイルの説明を追加するためのマクロ．
//! @param VARIABLE 変数名．
//! @param TABLE テーブル名．
//! @param DESCRIPTION 説明．
#define DESIGNLAB_TOML11_ADD_DESCRIPTION(VARIABLE, TABLE, DESCRIPTION)		\
const ::designlab::toml_func::Toml11Description VARIABLE{TABLE, sjis_to_utf8(DESCRIPTION)} 

//! @def DESIGNLAB_ADD_FILE_DESCRIPTION
//! @brief ファイルの説明を追加したくない場合には，このマクロで追加する．
//! @param VARIABLE 変数名．
//! @param TABLE テーブル名．
#define DESIGNLAB_TOML11_ADD_NO_DESCRIPTION(VARIABLE, TABLE)		\
const ::designlab::toml_func::Toml11Description VARIABLE{TABLE, ""} 


//! @def DESIGNLAB_TOML11_SERIALIZE
//! @brief tomlファイルのシリアライズ/デシリアライズを行うためのマクロ．
//! @n TOML11_DEFINE_CONVERSION_NON_INTRUSIVEをラッパしたもの．
//! @n もともとのほうではenum型を取り扱うことができなかったが，このマクロでは取り扱うことができる．
//! @n また，クラスの説明を追加することができる．
//! @n 注意点としてこのクラスを使用する場合は必ずDESIGNLAB_TOML11_DESCRIPTION_CLASSを用意する必要がある．
//! @details 以下のように使用する．
//! @code 
//! enum class SampleEnum : int
//! {
//!     A,
//!     B,
//!     C,
//! }
//! 
//! class Sameple final
//! {
//! public:
//!     int data{ 0 };
//!     std::string str{ "This is String"};
//!     SampleEnum enum_data{ SampleEnum::A };
//! };
//! 
//! DESIGNLAB_TOML11_DESCRIPTION_CLASS(Sample)
//! {
//!     DESIGNLAB_TOML11_NO_FILE_DESCRIPTION();
//!     DESIGNLAB_TOML11_ADD_DESCRIPTION(data, "number", "This is data");
//!     DESIGNLAB_TOML11_ADD_DESCRIPTION(str, DESIGNLAB_TOML11_NO_TABLE, "This is str");
//!     DESIGNLAB_TOML11_ADD_DESCRIPTION(enum_data, "enum", "This is enum_data");
//! };
//! 
//! DESIGNLAB_TOML11_SERIALIZE(Sample, data, str, enum_data);
//! @endcode
//! @param NAME クラス名．クラスの型を指定する．
//! @param ... クラスのメンバ変数．過不足なく，全て指定する必要がある．可変長引数なので複数指定することができる．
#define DESIGNLAB_TOML11_SERIALIZE(NAME, ...)                                                     \
namespace toml                                                                                    \
{                                                                                                 \
template<>                                                                                        \
struct from<NAME>                                                                                 \
{                                                                                                 \
    static_assert(std::is_class<NAME>::value,                                                     \
        "第1引数はクラスか構造体である必要があります．" );                                        \
	static_assert(std::is_default_constructible<NAME>::value,                                     \
        "第1引数はデフォルトコンストラクタを持つ必要があります．");                               \
                                                                                                  \
    template<typename C, template<typename ...> class T,                                          \
             template<typename ...> class A>                                                      \
    static NAME from_toml(basic_value<C, T, A>& v)                                          \
    {                                                                                             \
        NAME obj;                                                                                 \
        NAME##Description desc;															          \
        TOML11_FOR_EACH_VA_ARGS(DESIGNLAB_SUB_MACRO_FIND_MEMBER_VARIABLE_FROM_VALUE, __VA_ARGS__) \
        return obj;                                                                               \
    }                                                                                             \
};                                                                                                \
                                                                                                  \
template<>                                                                                        \
struct into<NAME>																					\
{																									\
    static value into_toml(const NAME& obj)															\
    {																								\
        ::toml::basic_value<toml::preserve_comments, std::map> v = ::toml::table{};					\
																									\
        NAME##Description desc;																		\
																									\
        for(const auto i : desc.file_description_vec)												\
		{																							\
			v.comments().push_back(i);																\
		}																							\
																									\
        for(int i = 0; i < desc.table_name_description_vec.size(); ++i)								\
        {																							\
			v[desc.table_name_description_vec[i]] = ::toml::table{};								\
			v[desc.table_name_description_vec[i]].comments().										\
				push_back(desc.table_name_description_vec[i + 1]);									\
			++i;																					\
        }																							\
																									\
		TOML11_FOR_EACH_VA_ARGS(DESIGNLAB_SUB_MACRO_ASSIGN_MEMBER_VARIABLE_TO_VALUE, __VA_ARGS__)	\
		TOML11_FOR_EACH_VA_ARGS(DESIGNLAB_SUB_MACRO_ADD_COMMENT, __VA_ARGS__)						\
		return v;																					\
	}																								\
};																									\
\
} /* namesppppac toml */


#endif // DESIGNLAB_TOML_SERIALIZE_MACRO_H_