#pragma once
#include <vector>

//��{�I�Ȍv�Z���s���֐�
namespace my_math
{
	constexpr float MY_FLT_PI = 3.141592653589793f;		//!< float�^�̉~����
	constexpr double MY_DBL_PI = 3.141592653589793;		//!< double�^�̉~����
	constexpr double DBL_ALLOWABLE_ERROR = 0.001;					//!< ����ȏ㏬�����l��0�Ƃ݂Ȃ��Dallowable error�C���e�덷�̂��ƁD0.1�Ƃ��C�傫�Ȓl�͔񐄏��D
	constexpr float ALLOWABLE_ERROR = (float)DBL_ALLOWABLE_ERROR;	//!< ����ȏ㏬�����l��0�Ƃ݂Ȃ��Dallowable error�C���e�덷�̂��ƁD0.1�Ƃ��C�傫�Ȓl�͔񐄏��D

	//! @brief C++�ɂ����āC�������m�̌v�Z�͌덷���o�Ă��܂��D�덷���݂Œl�������������ׂ�D
	//! @param [in] _num1 ��r���鐔��1�� 
	//! @param [in] _num2 ��r���鐔��2��
	//! @return bool �������Ȃ��true 
	constexpr bool isEqual(const float _num1, const float _num2) 
	{
		const float dif = _num1 - _num2;
		if (dif > 0) { return (dif <= ALLOWABLE_ERROR); }
		else { return (dif >= -ALLOWABLE_ERROR); }
	}

	//! @brief C++�ɂ����āC�������m�̌v�Z�͌덷���o�Ă��܂��D�덷���݂Œl�������������ׂ�D
	//! @param [in] _num1 ��r���鐔��1�� 
	//! @param [in] _num2 ��r���鐔��2��
	//! @return bool �������Ȃ��true 
	constexpr bool isEqual(const double _num1, const double _num2)
	{
		const double dif = _num1 - _num2;
		if (dif > 0) { return (dif <= DBL_ALLOWABLE_ERROR); }
		else { return (dif >= -DBL_ALLOWABLE_ERROR); }
	}

	//! @brief 2�悵���l��Ԃ��֐��D
	//! @param [in] _num 2�悷�鐔�D
	//! @return double 2�悵���l�D 
	constexpr double squared(const double _num) { return _num * _num; }	

	//! @brief 2�悵���l��Ԃ��֐��D
	//! @param [in] _num 2�悷�鐔�D
	//! @return float 2�悵���l�D 
	constexpr float squared(const float _num) { return _num * _num; }

	//! @brief 2�悵���l��Ԃ��֐��D
	//! @param [in] _num 2�悷�鐔�D
	//! @return int 2�悵���l�D 
	constexpr int squared(const int _num) { return _num * _num; }

	//! @brief 2�悵���l��Ԃ��֐��D
	//! @param [in] _num 2�悷�鐔�D
	//! @return char 2�悵���l�D 
	constexpr char squared(const char _num) { return _num * _num; }

	//! @brief 2�悵���l��Ԃ��֐��D
	//! @param [in] _num 2�悷�鐔�D
	//! @return short 2�悵���l�D
	constexpr short squared(const short _num) { return _num * _num; }


	double generateRandomNumber(const double _min, const double _max);	//�w�肵���͈͓��̗����𐶐�����.
	float generateRandomNumber(const float _min, const float _max);		//�w�肵���͈͓��̗����𐶐�����.
	int generateRandomNumber(const int _min, const int _max);			//�w�肵���͈͓��̗����𐶐�����.

	constexpr double convertRadToDeg(const double _rad) { return _rad * 180.0 / MY_DBL_PI; };
	constexpr float convertRadToDeg(const float _rad) { return _rad * 180.0f / MY_FLT_PI; };

	constexpr double convertDegToRad(const double _deg) { return _deg * MY_DBL_PI / 180.0; }
	constexpr float convertDegToRad(const float _deg) { return _deg * MY_FLT_PI / 180.0f; }
}
