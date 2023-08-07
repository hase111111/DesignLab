#pragma once

#include <vector>

//! @namespacse my_math ��{�I�Ȍv�Z���s���֐����܂Ƃ߂����O��ԁD
//! @date 2023/08/06
//! @auther ���J��
//! @brief ��{�I�Ȍv�Z���s���֐����܂Ƃ߂����O��ԁD
namespace my_math
{
	constexpr double MY_DBL_PI = 3.141592653589793;		//!< double�^�̉~����
	constexpr float MY_FLT_PI = 3.141592653589793f;		//!< float�^�̉~����

	constexpr double DBL_ALLOWABLE_ERROR = 0.001;								//!< ����ȏ㏬�����l��0�Ƃ݂Ȃ��Dallowable error�C���e�덷�̂��ƁD0.1�Ƃ��C�傫�Ȓl�͔񐄏��D
	constexpr float ALLOWABLE_ERROR = static_cast<float>(DBL_ALLOWABLE_ERROR);	//!< ����ȏ㏬�����l��0�Ƃ݂Ȃ��Dallowable error�C���e�덷�̂��ƁD0.1�Ƃ��C�傫�Ȓl�͔񐄏��D


	//! @brief C++�ɂ����āC�������m�̌v�Z�͌덷���o�Ă��܂��D�덷���݂Œl�������������ׂ�D
	//! @param [in] num1 ��r���鐔��1�� 
	//! @param [in] num2 ��r���鐔��2��
	//! @return bool �������Ȃ��true 
	constexpr bool isEqual(const float num1, const float num2)
	{
		const float dif = num1 - num2;
		if (dif > 0) { return (dif <= ALLOWABLE_ERROR); }
		else { return (dif >= -ALLOWABLE_ERROR); }
	}

	//! @brief C++�ɂ����āC�������m�̌v�Z�͌덷���o�Ă��܂��D�덷���݂Œl�������������ׂ�D
	//! @param [in] num1 ��r���鐔��1�� 
	//! @param [in] num2 ��r���鐔��2��
	//! @return bool �������Ȃ��true 
	constexpr bool isEqual(const double num1, const double num2)
	{
		const double dif = num1 - num2;
		if (dif > 0) { return (dif <= DBL_ALLOWABLE_ERROR); }
		else { return (dif >= -DBL_ALLOWABLE_ERROR); }
	}

	//! @brief 2�悵���l��Ԃ��֐��D
	//! @param [in] num 2�悷�鐔�D
	//! @return double 2�悵���l�D 
	constexpr double squared(const double num) { return num * num; }

	//! @brief 2�悵���l��Ԃ��֐��D
	//! @param [in] num 2�悷�鐔�D
	//! @return float 2�悵���l�D 
	constexpr float squared(const float num) { return num * num; }

	//! @brief 2�悵���l��Ԃ��֐��D
	//! @param [in] num 2�悷�鐔�D
	//! @return int 2�悵���l�D 
	constexpr int squared(const int num) { return num * num; }

	//! @brief 2�悵���l��Ԃ��֐��D
	//! @param [in] num 2�悷�鐔�D
	//! @return char 2�悵���l�D 
	constexpr char squared(const char num) { return num * num; }

	//! @brief 2�悵���l��Ԃ��֐��D
	//! @param [in] num 2�悷�鐔�D
	//! @return short 2�悵���l�D
	constexpr short squared(const short num) { return num * num; }


	//! @brief �w�肵���͈͓��̗����𐶐�����D
	//! @param [in] min �����̍ŏ��l�D
	//! @param [in] max �����̍ő�l�D
	//! @return double �������������D
	double generateRandomNumber(const double min, const double max);

	//! @brief �w�肵���͈͓��̗����𐶐�����D
	//! @param [in] min �����̍ŏ��l�D
	//! @param [in] max �����̍ő�l�D
	//! @return float �������������D
	float generateRandomNumber(const float min, const float max);

	//! @brief �w�肵���͈͓��̗����𐶐�����D
	//! @param [in] min �����̍ŏ��l�D
	//! @param [in] max �����̍ő�l�D
	//! @return int �������������D
	int generateRandomNumber(const int min, const int max);


	//! @brief �p�x��rad����deg�ɕϊ�����֐��D
	//! @param [in] rad �p�x[rad]�D
	//! @return double �p�x[deg]�D
	constexpr double convertRadToDeg(const double rad) { return rad * 180.0 / MY_DBL_PI; };

	//! @brief �p�x��rad����deg�ɕϊ�����֐��D
	//! @param [in] rad �p�x[rad]�D
	//! @return float �p�x[deg]�D
	constexpr float convertRadToDeg(const float rad) { return rad * 180.0f / MY_FLT_PI; };

	//! @brief �p�x��deg����rad�ɕϊ�����֐��D
	//! @param [in] deg �p�x[deg]�D
	//! @return double �p�x[rad]�D
	constexpr double convertDegToRad(const double deg) { return deg * MY_DBL_PI / 180.0; }

	//! @brief �p�x��deg����rad�ɕϊ�����֐��D
	//! @param [in] deg �p�x[deg]�D
	//! @return float �p�x[rad]�D
	constexpr float convertDegToRad(const float deg) { return deg * MY_FLT_PI / 180.0f; }

	float limitRangeAngle(const float angle);

}	// namespace my_math


//! @file my_math.h
//! @date 2023/08/06
//! @auther ���J��
//! @brief ��{�I�Ȍv�Z���s���֐����܂Ƃ߂����O��ԁD
//! @n �s�� : @lineinfo