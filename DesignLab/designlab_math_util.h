//! @file designlab_math_util.h
//! @brief ��{�I�Ȍv�Z���s���֐��D


#ifndef DESIGNLAB_MATH_UTIL_H_
#define DESIGNLAB_MATH_UTIL_H_


#include <string>
#include <vector>

#include "cassert_define.h"


//! @namespace designlab
//! @brief DesignLab�̖��O���
//! @details ���̃v���W�F�N�g�ō쐬�����֐��͑S�Ă��̖��O��ԓ��ɓ����D
//! @n �O���[�o���ȋ�ԂɊ֐�������ƁC���O�̏Փ˂��N����\��������D
namespace designlab 
{
	//! @namespacse math_util ��{�I�Ȍv�Z���s���֐����܂Ƃ߂����O��ԁD
	//! @brief ��{�I�Ȍv�Z���s���֐����܂Ƃ߂����O��ԁD
	namespace math_util 
	{
		constexpr double kDoublePi = 3.141592653589793;		//!< double�^�̉~����
		constexpr float kFloatPi = 3.141592653589793f;		//!< float�^�̉~����

		constexpr double kDoubleAllowableError = 0.001;								//!< ����ȏ㏬�����l��0�Ƃ݂Ȃ��Dallowable error�C���e�덷�̂��ƁD0.1�Ƃ��C�傫�Ȓl�͔񐄏��D
		constexpr float kAllowableError = static_cast<float>(kDoubleAllowableError);	//!< ����ȏ㏬�����l��0�Ƃ݂Ȃ��Dallowable error�C���e�덷�̂��ƁD0.1�Ƃ��C�傫�Ȓl�͔񐄏��D


		//! @brief C++�ɂ����āC�������m�̌v�Z�͌덷���o�Ă��܂��D�덷���݂Œl�������������ׂ�D
		//! @param [in] num1 ��r���鐔��1�� 
		//! @param [in] num2 ��r���鐔��2��
		//! @return bool �������Ȃ��true 
		constexpr bool IsEqual(const float num1, const float num2)
		{
			const float dif = num1 - num2;
			if (dif > 0) { return (dif <= kAllowableError); }
			else { return (dif >= -kAllowableError); }
		}

		//! @brief C++�ɂ����āC�������m�̌v�Z�͌덷���o�Ă��܂��D�덷���݂Œl�������������ׂ�D
		//! @param [in] num1 ��r���鐔��1�� 
		//! @param [in] num2 ��r���鐔��2��
		//! @return bool �������Ȃ��true 
		constexpr bool IsEqual(const double num1, const double num2)
		{
			const double dif = num1 - num2;
			if (dif > 0) { return (dif <= kDoubleAllowableError); }
			else { return (dif >= -kDoubleAllowableError); }
		}

		//! @brief 2�悵���l��Ԃ��֐��D
		//! @n �����^��C�����^�݂̂�z�肵�č���Ă���̂ŁC���̌^�Ŏg���ƃG���[���o�邩���D
		//! @param [in] num 2�悷�鐔�D
		//! @return T 2�悵���l�D 
		template <typename T>
		constexpr T Squared(const T num) { return num * num; }

		//! @brief �ڕW�l�ɒl���߂Â���֐��D
		//! @n �K���ɍ���Ă���C���`�ł��Ȃ��C�`��p�Ȃ̂Ōv�Z�Ɏg�������Ȃ��蒼������
		//! @param [in] current ���݂̒l�D
		//! @param [in] target �ڕW�l�D
		//! @param [in] rate �߂Â��銄���D0 �` 1�̒l�����D
		//! @return T �߂Â����l�D
		template <typename T>
		T ApproachTarget(const T& current,const T& target, float rate) 
		{
			assert(0 <= rate);
			assert(rate <= 1);	//0 <= rate <= 1 �łȂ���΂Ȃ�Ȃ�

			if (current == target) { return current; }

			return current * (1 - rate) + target * rate;
		}

		//! @brief �w�肵���͈͓��̗����𐶐�����D
		//! @param [in] min �����̍ŏ��l�D
		//! @param [in] max �����̍ő�l�D
		//! @return double �������������D
		double GenerateRandomNumber(double min, double max);

		//! @brief �w�肵���͈͓��̗����𐶐�����D
		//! @param [in] min �����̍ŏ��l�D
		//! @param [in] max �����̍ő�l�D
		//! @return float �������������D
		float GenerateRandomNumber(float min, float max);

		//! @brief �w�肵���͈͓��̗����𐶐�����D
		//! @param [in] min �����̍ŏ��l�D
		//! @param [in] max �����̍ő�l�D
		//! @return int �������������D
		int GenerateRandomNumber(int min, int max);


		//! @brief �p�x��rad����deg�ɕϊ�����֐��D
		//! @param [in] rad �p�x[rad]�D
		//! @return double �p�x[deg]�D
		constexpr double ConvertRadToDeg(const double rad) { return rad * 180.0 / kDoublePi; };

		//! @brief �p�x��rad����deg�ɕϊ�����֐��D
		//! @param [in] rad �p�x[rad]�D
		//! @return float �p�x[deg]�D
		constexpr float ConvertRadToDeg(const float rad) { return rad * 180.0f / kFloatPi; };

		//! @brief �p�x��deg����rad�ɕϊ�����֐��D
		//! @param [in] deg �p�x[deg]�D
		//! @return double �p�x[rad]�D
		constexpr double ConvertDegToRad(const double deg) { return deg * kDoublePi / 180.0; }

		//! @brief �p�x��deg����rad�ɕϊ�����֐��D
		//! @param [in] deg �p�x[deg]�D
		//! @return float �p�x[rad]�D
		constexpr float ConvertDegToRad(const float deg) { return deg * kFloatPi / 180.0f; }


		[[deprecated]] 
		float limitRangeAngle(const float angle);


		constexpr int kDigit = 3;	//!< �����_�ȉ��̌����D
		constexpr int kWidth = 10;	//!< ������̕��D

		//! @brief �����𕶎���ɕϊ�����֐��D
		//! @n C++ �ł� C �̃t�H�[�}�b�g�̂悤�� %3.3f �Ƃ��ŏ����𕶎���ɕϊ��ł��Ȃ����ߎ��삷��
		//! @param [in] num �ϊ����鏬���D
		//! @param [in] digit �����_�ȉ��̌����D
		//! @param [in] width ������̕��D
		//! @return std::string �ϊ�����������D
		std::string ConvertFloatToString(const float num, const int digit = kDigit, const int width = kWidth);

		//! @brief �����𕶎���ɕϊ�����֐��D
		//! @n C++ �ł� C �̃t�H�[�}�b�g�̂悤�� %3.3f �Ƃ��ŏ����𕶎���ɕϊ��ł��Ȃ����ߎ��삷��
		//! @param [in] num �ϊ����鏬���D
		//! @param [in] digit �����_�ȉ��̌����D
		//! @param [in] width ������̕��D
		//! @return std::string �ϊ�����������D
		std::string ConvertDoubleToString(const double num, const int digit = kDigit, const int width = kWidth);

	}	// namespace math_util

}	// namespace designlab


#endif	// DESIGNLAB_MATH_UTIL_H_