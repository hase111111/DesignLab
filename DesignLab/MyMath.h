#pragma once

//��{�I�Ȍv�Z���s���֐�
namespace my_math
{
	//2�悵���l��Ԃ��֐��D���W���[�Ȍ^�̓I�[�o�[���[�h�����D
	constexpr double squared(const double _num) { return _num * _num; }
	constexpr float squared(const float _num) { return _num * _num; }
	constexpr int squared(const int _num) { return _num * _num; }
	constexpr char squared(const char _num) { return _num * _num; }
	constexpr short squared(const short _num) { return _num * _num; }
}