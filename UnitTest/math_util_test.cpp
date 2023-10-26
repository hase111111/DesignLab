#include "pch.h"

#include "../DesignLab/designlab_math_util.h"
#include "../DesignLab/designlab_math_util.cpp"


namespace dlm = designlab::math_util;

namespace dl_math_test
{
	TEST(MathUtilTest, IsEqualTestFloat)
	{
		//float�^�̃I�[�o�[���[�h

		//TRUE
		EXPECT_TRUE(dlm::IsEqual(2.0f, 2.0f));
		EXPECT_TRUE(dlm::IsEqual(-9.0f, -9.0f));
		EXPECT_TRUE(dlm::IsEqual(129.0f, 129.0f));
		EXPECT_TRUE(dlm::IsEqual(-9125.0f, -9125.0f));

		//FALSE
		EXPECT_FALSE(dlm::IsEqual(9.0f, 1.0f));
		EXPECT_FALSE(dlm::IsEqual(9.0f, -9.0f));
		EXPECT_FALSE(dlm::IsEqual(25.0f, 64.0f));
		EXPECT_FALSE(dlm::IsEqual(-91.0f, -26.0f));

		//�����_�ȉ�3���܂ł̌덷�����e����D�傫���ꍇ
		EXPECT_TRUE(dlm::IsEqual(5.0f, 5.0f + 0.00001f));
		EXPECT_TRUE(dlm::IsEqual(5.0f, 5.0f + 0.0001f));
		EXPECT_FALSE(dlm::IsEqual(5.0f, 5.0f + 0.001f));
		EXPECT_FALSE(dlm::IsEqual(5.0f, 5.0f + 0.01f));
		EXPECT_FALSE(dlm::IsEqual(5.0f, 5.0f + 0.1f));

		//�������ꍇ
		EXPECT_TRUE(dlm::IsEqual(5.0f, 5.0f - 0.00001f));
		EXPECT_TRUE(dlm::IsEqual(5.0f, 5.0f - 0.0001f));
		EXPECT_FALSE(dlm::IsEqual(5.0f, 5.0f - 0.001f));
		EXPECT_FALSE(dlm::IsEqual(5.0f, 5.0f - 0.01f));
		EXPECT_FALSE(dlm::IsEqual(5.0f, 5.0f - 0.1f));
	}

	TEST(MathUtilTest, IsEqualTestDouble)
	{
		//double�^�̃I�[�o�[���[�h

		//TRUE
		EXPECT_TRUE(dlm::IsEqual(2.0, 2.0));
		EXPECT_TRUE(dlm::IsEqual(-9.0, -9.0));
		EXPECT_TRUE(dlm::IsEqual(129.0, 129.0));
		EXPECT_TRUE(dlm::IsEqual(-628.0, -628.0));
		EXPECT_TRUE(dlm::IsEqual(-9125.0, -9125.0));

		//FALSE
		EXPECT_FALSE(dlm::IsEqual(9.0, 1.0));
		EXPECT_FALSE(dlm::IsEqual(9.0, -9.0));
		EXPECT_FALSE(dlm::IsEqual(25.0, 64.0));
		EXPECT_FALSE(dlm::IsEqual(-91.0, -26.0));
		EXPECT_FALSE(dlm::IsEqual(0.0, -5.0));

		//�����_�ȉ�3���܂ł̌덷�����e����D�傫���ꍇ
		EXPECT_TRUE(dlm::IsEqual(5.0, 5.0 + 0.00001));
		EXPECT_TRUE(dlm::IsEqual(5.0, 5.0 + 0.0001));
		EXPECT_FALSE(dlm::IsEqual(5.0, 5.0 + 0.001));
		EXPECT_FALSE(dlm::IsEqual(5.0, 5.0 + 0.01));
		EXPECT_FALSE(dlm::IsEqual(5.0, 5.0 + 0.1));

		//�������ꍇ
		EXPECT_TRUE(dlm::IsEqual(5.0, 5.0 - 0.00001));
		EXPECT_TRUE(dlm::IsEqual(5.0, 5.0 - 0.0001));
		EXPECT_FALSE(dlm::IsEqual(5.0, 5.0 - 0.001));
		EXPECT_FALSE(dlm::IsEqual(5.0, 5.0 - 0.01));
		EXPECT_FALSE(dlm::IsEqual(5.0, 5.0 - 0.1));
	}

	TEST(MathUtilTest, Squared)
	{
		//int�^
		EXPECT_EQ(dlm::Squared(-13), 169);
		EXPECT_EQ(dlm::Squared(4), 16);
		EXPECT_EQ(dlm::Squared(0), 0);

		//double�^
		EXPECT_TRUE(dlm::IsEqual(dlm::Squared(4.8), 23.04));
		EXPECT_TRUE(dlm::IsEqual(dlm::Squared(125.2), 15675.04));
		EXPECT_TRUE(dlm::IsEqual(dlm::Squared(0.1), 0.01));

		//float�^
		EXPECT_TRUE(dlm::IsEqual(dlm::Squared(4.8f), 23.04f));
		EXPECT_TRUE(dlm::IsEqual(dlm::Squared(125.2f), 15675.04f));
		EXPECT_TRUE(dlm::IsEqual(dlm::Squared(0.1f), 0.01f));

		//char�^
		EXPECT_EQ(dlm::Squared<char>(11), char{121});
		EXPECT_EQ(dlm::Squared<char>(-7), char{49});

		//unsigned int�^
		EXPECT_EQ(dlm::Squared<unsigned int>(13), 169);
		EXPECT_EQ(dlm::Squared<unsigned int>(1429), 2042041);
	}

	TEST(MathUtilTest, CanMakeTriangle) 
	{
		//TRUE
		EXPECT_TRUE(dlm::CanMakeTriangle(1.0, 1.0, 1.0));
		EXPECT_TRUE(dlm::CanMakeTriangle(1.0, 1.0, 2.0));
		EXPECT_TRUE(dlm::CanMakeTriangle(1.0, 2.0, 1.0));
		EXPECT_TRUE(dlm::CanMakeTriangle(2.0, 1.0, 1.0));
		EXPECT_TRUE(dlm::CanMakeTriangle(2.0, 2.0, 2.0));
		EXPECT_TRUE(dlm::CanMakeTriangle(2.0, 2.0, 3.0));
		EXPECT_TRUE(dlm::CanMakeTriangle(2.0, 3.0, 2.0));
		EXPECT_TRUE(dlm::CanMakeTriangle(3.0, 2.0, 2.0));
		EXPECT_TRUE(dlm::CanMakeTriangle(3.0, 3.0, 3.0));
		EXPECT_TRUE(dlm::CanMakeTriangle(3.0, 3.0, 4.0));
		EXPECT_TRUE(dlm::CanMakeTriangle(3.0, 4.0, 3.0));
		EXPECT_TRUE(dlm::CanMakeTriangle(4.0, 3.0, 3.0));
		EXPECT_TRUE(dlm::CanMakeTriangle(4.0, 4.0, 4.0));
		EXPECT_TRUE(dlm::CanMakeTriangle(4.0, 4.0, 5.0));
		EXPECT_TRUE(dlm::CanMakeTriangle(4.0, 5.0, 4.0));
		EXPECT_TRUE(dlm::CanMakeTriangle(5.0, 4.0, 4.0));
		EXPECT_TRUE(dlm::CanMakeTriangle(5.0, 5.0, 5.0));
	}
}