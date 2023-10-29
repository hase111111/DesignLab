#include "pch.h"

#include "../DesignLab/designlab_math_util.h"


namespace dlm = designlab::math_util;


namespace designlab::test::common::math
{
	TEST(MathUtilTest, GenerateRandomNumberTestNumInRangeCaseOfDouble)
	{
		// ������Ԃ����\�b�h�̃e�X�g�D
		// �����Ȃ̂ŁC�e�X�g�͂��܂�Ӗ����Ȃ��C������...�悢���@�͂Ȃ񂶂��ق��D
		// �Ƃ肠�����C���s�񐔂𑝂₵�āC�m�F������j�ł����܂��D

		constexpr int kTestNum = 10000;	//���s��
		constexpr double kMin = -10.45;
		constexpr double kMax = 234.47;

		for (int i = 0; i < kTestNum; ++i)
		{
			const double _res = dlm::GenerateRandomNumber(kMin, kMax);

			// LT = Less Than <
			// GT = Greater Than >

			EXPECT_LT(kMin, _res) << "�ŏ��l��������Ă��܂��D";
			EXPECT_GT(kMax, _res) << "�ő�l�������Ă��܂��D";
		}
	}

	TEST(MathUtilTest, GenerateRandomNumberTestNumInRangeCaseOfFloat)
	{
		// ������Ԃ����\�b�h�̃e�X�g�D
		// �����Ȃ̂ŁC�e�X�g�͂��܂�Ӗ����Ȃ��C������...�悢���@�͂Ȃ񂶂��ق��D
		// �Ƃ肠�����C���s�񐔂𑝂₵�āC�m�F������j�ł����܂��D

		constexpr int kTestNum = 10000;	//���s��
		constexpr float kMin = -10.45f;
		constexpr float kMax = 234.47f;

		for (int i = 0; i < kTestNum; ++i)
		{
			const float _res = dlm::GenerateRandomNumber(kMin, kMax);

			// LT = Less Than <
			// GT = Greater Than >

			EXPECT_LT(kMin, _res) << "�ŏ��l��������Ă��܂��D";
			EXPECT_GT(kMax, _res) << "�ő�l�������Ă��܂��D";
		}
	}

	TEST(MathUtilTest, GenerateRandomNumberTestNumInRangeCaseOfInt)
	{
		// ������Ԃ����\�b�h�̃e�X�g�D
		// �����Ȃ̂ŁC�e�X�g�͂��܂�Ӗ����Ȃ��C������...�悢���@�͂Ȃ񂶂��ق��D
		// �Ƃ肠�����C���s�񐔂𑝂₵�āC�m�F������j�ł����܂��D

		constexpr int kTestNum = 10000;	//���s��
		constexpr int kMin = -10;
		constexpr int kMax = 234;

		for (int i = 0; i < kTestNum; ++i)
		{
			const int _res = dlm::GenerateRandomNumber(kMin, kMax);

			// LT = Less Than <
			// GT = Greater Than >

			EXPECT_LT(kMin, _res) << "�ŏ��l��������Ă��܂��D";
			EXPECT_GT(kMax, _res) << "�ő�l�������Ă��܂��D";
		}
	}
}