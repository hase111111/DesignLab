#include "pch.h"

#include "../DesignLab/designlab_vector3.h"


namespace dl_vec_test
{
	TEST(Vector3, EqualityOperator)
	{
		//�������Z�q�̃e�X�g

		EXPECT_TRUE(designlab::Vector3(10, -5, 0) == designlab::Vector3(10, -5, 0));
		EXPECT_TRUE(designlab::Vector3(-20, 60, 10) == designlab::Vector3(-20, 60, 10));
		EXPECT_TRUE(designlab::Vector3(0.4f, 6.6f, -7.8f) == designlab::Vector3(0.4f, 6.6f, -7.8f));
		EXPECT_TRUE(designlab::Vector3(100000, -5200, 62000) == designlab::Vector3(100000, -5200, 62000));

		EXPECT_FALSE(designlab::Vector3(10, -5, 0) != designlab::Vector3(10, -5, 0));
		EXPECT_FALSE(designlab::Vector3(-20, 60, 10) != designlab::Vector3(-20, 60, 10));
		EXPECT_FALSE(designlab::Vector3(0.4f, 6.6f, -7.8f) != designlab::Vector3(0.4f, 6.6f, -7.8f));
		EXPECT_FALSE(designlab::Vector3(100000, -5200, 62000) != designlab::Vector3(100000, -5200, 62000));


		EXPECT_FALSE(designlab::Vector3(4, 8, -9) == designlab::Vector3(-9, 3, 1));
		EXPECT_FALSE(designlab::Vector3(10, -20, 53) == designlab::Vector3(-34, 31, -95));
		EXPECT_FALSE(designlab::Vector3(0.7f, -4.1f, 6.7f) == designlab::Vector3(5.2f, 6.8f, 9.1f));
		EXPECT_FALSE(designlab::Vector3(2500, 71200, -91200) == designlab::Vector3(-9900, 250000, 23400));

		EXPECT_TRUE(designlab::Vector3(4, 8, -9) != designlab::Vector3(-9, 3, 1));
		EXPECT_TRUE(designlab::Vector3(10, -20, 53) != designlab::Vector3(-34, 31, -95));
		EXPECT_TRUE(designlab::Vector3(0.7f, -4.1f, 6.7f) != designlab::Vector3(5.2f, 6.8f, 9.1f));
		EXPECT_TRUE(designlab::Vector3(2500, 71200, -91200) != designlab::Vector3(-9900, 250000, 23400));


		EXPECT_TRUE(designlab::Vector3(10, -5, 0) != designlab::Vector3(13, -5, 0));
		EXPECT_TRUE(designlab::Vector3(10, -5, 0) != designlab::Vector3(10, 55, 0));
		EXPECT_TRUE(designlab::Vector3(10, -5, 0) != designlab::Vector3(10, -5, 8));
	}

	TEST(Vector3, ComparisonOperator)
	{
		//��r���Z�q�̃e�X�g�D��r���Z�q�̓x�N�g���̒�����p���Ĕ�r���s���D

		EXPECT_TRUE(designlab::Vector3(0, 0, 0) < designlab::Vector3(5, 10, -6));
		EXPECT_TRUE(designlab::Vector3(2, -6, 3) < designlab::Vector3(56, 20, -94));
		EXPECT_TRUE(designlab::Vector3(5, 10, -6) > designlab::Vector3(0, 0, 0));
		EXPECT_TRUE(designlab::Vector3(56, 20, -94) > designlab::Vector3(2, -6, 3));

		EXPECT_TRUE(designlab::Vector3(0, 0, 0) <= designlab::Vector3(5, 10, -6));
		EXPECT_TRUE(designlab::Vector3(2, -6, 3) <= designlab::Vector3(56, 20, -94));
		EXPECT_TRUE(designlab::Vector3(5, 10, -6) >= designlab::Vector3(0, 0, 0));
		EXPECT_TRUE(designlab::Vector3(56, 20, -94) >= designlab::Vector3(2, -6, 3));

		EXPECT_TRUE(designlab::Vector3(-23, 85, 91) <= designlab::Vector3(-23, 85, 91));
		EXPECT_TRUE(designlab::Vector3(-23, 85, 91) >= designlab::Vector3(-23, 85, 91));
	}

	TEST(Vector3, PlusOperator)
	{
		//�����Z
		EXPECT_EQ(designlab::Vector3(2, 2, 2), designlab::Vector3(1, 1, 0) + designlab::Vector3(1, 1, 2));
		EXPECT_EQ(designlab::Vector3(10, 0, 0), designlab::Vector3(5, 0, 0) + designlab::Vector3(5, 0, 0));
		EXPECT_EQ(designlab::Vector3(0, 10, 0), designlab::Vector3(0, 3, 0) + designlab::Vector3(0, 7, 0));
		EXPECT_EQ(designlab::Vector3(0, 0, 10), designlab::Vector3(0, 0, 6) + designlab::Vector3(0, 0, 4));
		EXPECT_EQ(designlab::Vector3(100, 100, 100), designlab::Vector3(60, 120, 200) + designlab::Vector3(40, -20, -100));
		EXPECT_EQ(designlab::Vector3(2.8f, 21.6f, 12.89f), designlab::Vector3(0.2f, 34.8f, 6.78f) + designlab::Vector3(2.6f, -13.2f, 6.11f));
	}

	TEST(Vector3, MinusOperator)
	{
		//�����Z
		EXPECT_EQ(designlab::Vector3(10, 6, 10), designlab::Vector3(15, 66, 1010) - designlab::Vector3(5, 60, 1000));
		EXPECT_EQ(designlab::Vector3(-5, 150, 50), designlab::Vector3(0, 210, -50) - designlab::Vector3(5, 60, -100));
		EXPECT_EQ(designlab::Vector3(10, 6, 10), designlab::Vector3(15, 66, 1010) - designlab::Vector3(5, 60, 1000));
	}

	TEST(Vector3, MultiplyOperator)
	{
		//�|���Z
		EXPECT_EQ(designlab::Vector3(10, 6, 21) * 10, designlab::Vector3(100, 60, 210));
		EXPECT_EQ(designlab::Vector3(-2, 7, 100) * -4, designlab::Vector3(8, -28, -400));
		EXPECT_EQ(10 * designlab::Vector3(10, 6, 21), designlab::Vector3(100, 60, 210));
		EXPECT_EQ(-4 * designlab::Vector3(-2, 7, 100), designlab::Vector3(8, -28, -400));

		EXPECT_EQ(designlab::Vector3(62, 28, -84) * 3.7f, designlab::Vector3(229.4f, 103.6f, -310.8f));
		EXPECT_EQ(3.7f * designlab::Vector3(62, 28, -84), designlab::Vector3(229.4f, 103.6f, -310.8f));

		EXPECT_EQ(designlab::Vector3(0.256f, -34.21f, 6.002f) * 25.3f, designlab::Vector3(6.4768f, -865.513f, 151.8506f));
		EXPECT_EQ(25.3f * designlab::Vector3(0.256f, -34.21f, 6.002f), designlab::Vector3(6.4768f, -865.513f, 151.8506f));
	}
}
