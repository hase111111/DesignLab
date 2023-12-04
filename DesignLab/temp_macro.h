#pragma once


#include <toml.hpp>


//�^���𕶎���ɂ���}�N��
#define DESIGNLAB_TYPE_NAME(x) #x

//���������������}�N��
#define DESIGNLAB_APPEND_STRING(x, y) x##y


#define DESIGNLAB_TOML_CONVERSION(NAME, ...) \
namespace toml  																\
{                                                                               \
    template<>                                                                  \
    struct from<NAME>                                                           \
    {                                                                           \
                                                                                \
        template<typename C, template<typename ...> class T,                    \
                 template<typename ...> class A>                                \
        static NAME from_toml([[maybe_unused]]const basic_value<C, T, A>& v)    \
        {                                                                       \
            NAME obj;                                                           \
            return obj;                                                         \
        }                                                                       \
    };                                                                          \
                                                                                \
    template<>                                                                  \
    struct into<NAME>                                                           \
    {                                                                           \
        static value into_toml([[maybe_unused]]const NAME& obj)                 \
        {                                                                       \
            ::toml::value v = ::toml::table{};                                  \
            return v;                                                           \
        }                                                                       \
    };                                                                          \
    static_assert(std::is_default_constructible<NAME>::value, DESIGNLAB_APPEND_STRING(DESIGNLAB_TYPE_NAME(NAME),"�^�̓f�t�H���g�R���X�g���N�^������܂���D"));   \
    static_assert(std::is_class<NAME>::value, DESIGNLAB_APPEND_STRING(DESIGNLAB_TYPE_NAME(NAME),"�^�̓N���X�ł��\���̂ł�����܂���C"));  \
} /* toml namespace */


//            TOML11_FOR_EACH_VA_ARGS(TOML11_FIND_MEMBER_VARIABLE_FROM_VALUE, __VA_ARGS__) \
//            TOML11_FOR_EACH_VA_ARGS(TOML11_ASSIGN_MEMBER_VARIABLE_TO_VALUE, __VA_ARGS__) \
