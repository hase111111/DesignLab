#include "pch.h"
#include "../DesignLab/MyVector2.h"

using namespace my_vec;

namespace SVector2Test
{
	// �P�����Z�q + �̃e�X�g
	TEST(SVector2Operator, UnaryPlusOperator)
	{
		SVector2 v1(1.0f, 2.0f);
		SVector2 v2 = +v1;
		EXPECT_EQ(v1, v2);

		SVector2 v3(-1.0f, -2.0f);
		SVector2 v4 = +v3;
		EXPECT_EQ(v3, v4);

		SVector2 v5(0.0f, 0.0f);
		SVector2 v6 = +v5;
		EXPECT_EQ(v5, v6);

		SVector2 v7(1.0f, -2.0f);
		SVector2 v8 = +v7;
		EXPECT_EQ(v7, v8);
	}

	//�P�����Z�q - �̃e�X�g
	TEST(SVector2Operator, UnaryMinusOperator)
	{
		SVector2 v1(1.0f, 2.0f);
		SVector2 v2 = -v1;
		EXPECT_EQ(v2.x, -v1.x);
		EXPECT_EQ(v2.y, -v1.y);

		SVector2 v3(-1.0f, -2.0f);
		SVector2 v4 = -v3;
		EXPECT_EQ(v4.x, -v3.x);
		EXPECT_EQ(v4.y, -v3.y);

		SVector2 v5(0.0f, 0.0f);
		SVector2 v6 = -v5;
		EXPECT_EQ(v6.x, -v5.x);
		EXPECT_EQ(v6.y, -v5.y);

		SVector2 v7(1.0f, -2.0f);
		SVector2 v8 = -v7;
		EXPECT_EQ(v8.x, -v7.x);
		EXPECT_EQ(v8.y, -v7.y);
	}
}