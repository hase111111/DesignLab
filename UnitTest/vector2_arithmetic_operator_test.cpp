#include "pch.h"

#include "../DesignLab/designlab_vector2.h"

using namespace designlab;

namespace dl_vec_test
{
	TEST(Vector2, PlusOperator)
	{
		//整数同士の足し算
		Vector2 v1(1, 2);
		Vector2 v2(3, 4);
		Vector2 v3 = v1 + v2;
		EXPECT_FLOAT_EQ(v3.x, 4);
		EXPECT_FLOAT_EQ(v3.y, 6);

		//整数との足し算
		Vector2 v4 = v1 + 1;
		EXPECT_FLOAT_EQ(v4.x, 2);
		EXPECT_FLOAT_EQ(v4.y, 3);

		//浮動小数点数同士の足し算
		Vector2 v5(0.4f, 0.9f);
		Vector2 v6(11.7f, 8.3f);
		Vector2 v7 = v5 + v6;
		EXPECT_FLOAT_EQ(v7.x, 12.1f);
		EXPECT_FLOAT_EQ(v7.y, 9.2f);

		//浮動小数点数との足し算
		Vector2 v8 = v5 + 1.1f;
		EXPECT_FLOAT_EQ(v8.x, 1.5f);
		EXPECT_FLOAT_EQ(v8.y, 2.0f);

		//順番が逆でも同じ値になる
		Vector2 v9 = v1 + v2;
		Vector2 v10 = v2 + v1;
		EXPECT_FLOAT_EQ(v9.x, v10.x);
		EXPECT_FLOAT_EQ(v9.y, v10.y);

		//結合法則
		Vector2 v11 = v1 + (v2 + v3);
		Vector2 v12 = (v1 + v2) + v3;
		EXPECT_FLOAT_EQ(v11.x, v12.x);
		EXPECT_FLOAT_EQ(v11.y, v12.y);
	}

	TEST(Vector2, MinusOperator)
	{
		//整数同士の引き算
		Vector2 v1(1, 2);
		Vector2 v2(3, 4);
		Vector2 v3 = v1 - v2;
		EXPECT_FLOAT_EQ(v3.x, -2);
		EXPECT_FLOAT_EQ(v3.y, -2);

		//整数との引き算
		Vector2 v4 = v1 - 1;
		EXPECT_FLOAT_EQ(v4.x, 0);
		EXPECT_FLOAT_EQ(v4.y, 1);

		//浮動小数点数同士の引き算
		Vector2 v5(0.4f, 0.9f);
		Vector2 v6(11.7f, 8.3f);
		Vector2 v7 = v5 - v6;
		EXPECT_FLOAT_EQ(v7.x, -11.3f);
		EXPECT_FLOAT_EQ(v7.y, -7.4f);

		//浮動小数点数との引き算
		Vector2 v8 = v5 - 1.1f;
		EXPECT_FLOAT_EQ(v8.x, -0.7f);
		EXPECT_FLOAT_EQ(v8.y, -0.2f);

		//順番が逆でも同じ値になる
		Vector2 v9 = v1 - v2;
		Vector2 v10 = v2 - v1;
		EXPECT_FLOAT_EQ(v9.x, -v10.x);
		EXPECT_FLOAT_EQ(v9.y, -v10.y);

		//分配法則
		Vector2 v11 = v1 - (v2 - v3);
		Vector2 v12 = v1 - v2 + v3;
		EXPECT_FLOAT_EQ(v11.x, v12.x);
		EXPECT_FLOAT_EQ(v11.y, v12.y);
	}

	TEST(Vector2, MultipleOperator)
	{
		//整数との掛け算
		Vector2 v1(1, 2);
		Vector2 v2 = v1 * 2;
		EXPECT_FLOAT_EQ(v2.x, 2);
		EXPECT_FLOAT_EQ(v2.y, 4);

		//浮動小数点数との掛け算
		Vector2 v3 = v1 * 0.5f;
		EXPECT_FLOAT_EQ(v3.x, 0.5f);
		EXPECT_FLOAT_EQ(v3.y, 1.0f);

		//順番が逆でも同じ値になる
		Vector2 v4 = v1 * 2;
		Vector2 v5 = 2 * v1;
		EXPECT_FLOAT_EQ(v4.x, v5.x);
		EXPECT_FLOAT_EQ(v4.y, v5.y);

		//負の数との掛け算
		Vector2 v6 = v1 * -2;
		EXPECT_FLOAT_EQ(v6.x, -2);
		EXPECT_FLOAT_EQ(v6.y, -4);

		//負の浮動小数点数との掛け算
		Vector2 v7 = v1 * -0.5f;
		EXPECT_FLOAT_EQ(v7.x, -0.5f);
		EXPECT_FLOAT_EQ(v7.y, -1.0f);
	}

	TEST(Vector2, DivisionOperator)
	{
		//整数との割り算
		Vector2 v1(1, 2);
		Vector2 v2 = v1 / 2;
		EXPECT_FLOAT_EQ(v2.x, 0.5f);
		EXPECT_FLOAT_EQ(v2.y, 1);

		//浮動小数点数との割り算
		Vector2 v3 = v1 / 0.5f;
		EXPECT_FLOAT_EQ(v3.x, 2);
		EXPECT_FLOAT_EQ(v3.y, 4);

		//負の数との割り算
		Vector2 v4 = v1 / -2;
		EXPECT_FLOAT_EQ(v4.x, -0.5f);
		EXPECT_FLOAT_EQ(v4.y, -1);

		//負の浮動小数点数との割り算
		Vector2 v5 = v1 / -0.5f;
		EXPECT_FLOAT_EQ(v5.x, -2);
		EXPECT_FLOAT_EQ(v5.y, -4);
	}
}