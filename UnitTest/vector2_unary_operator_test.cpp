#include "pch.h"

#include "../DesignLab/designlab_vector2.h"


namespace dl_vec_test
{
	// �P�����Z�q + �̃e�X�g
	TEST(Vector2, UnaryPlusOperator)
	{
		designlab::Vector2 v1(1.0f, 2.0f);
		designlab::Vector2 v2 = +v1;
		EXPECT_EQ(v1, v2);

		designlab::Vector2 v3(-1.0f, -2.0f);
		designlab::Vector2 v4 = +v3;
		EXPECT_EQ(v3, v4);

		designlab::Vector2 v5(0.0f, 0.0f);
		designlab::Vector2 v6 = +v5;
		EXPECT_EQ(v5, v6);

		designlab::Vector2 v7(1.0f, -2.0f);
		designlab::Vector2 v8 = +v7;
		EXPECT_EQ(v7, v8);
	}

	//�P�����Z�q - �̃e�X�g
	TEST(Vector2, UnaryMinusOperator)
	{
		designlab::Vector2 v1(1.0f, 2.0f);
		designlab::Vector2 v2 = -v1;
		EXPECT_EQ(v2.x, -v1.x);
		EXPECT_EQ(v2.y, -v1.y);

		designlab::Vector2 v3(-1.0f, -2.0f);
		designlab::Vector2 v4 = -v3;
		EXPECT_EQ(v4.x, -v3.x);
		EXPECT_EQ(v4.y, -v3.y);

		designlab::Vector2 v5(0.0f, 0.0f);
		designlab::Vector2 v6 = -v5;
		EXPECT_EQ(v6.x, -v5.x);
		EXPECT_EQ(v6.y, -v5.y);

		designlab::Vector2 v7(1.0f, -2.0f);
		designlab::Vector2 v8 = -v7;
		EXPECT_EQ(v8.x, -v7.x);
		EXPECT_EQ(v8.y, -v7.y);
	}
}