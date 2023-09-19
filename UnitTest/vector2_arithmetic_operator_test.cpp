#include "pch.h"

#include "../DesignLab/designlab_vector2.h"

using namespace designlab;

namespace dl_vec_test
{
	TEST(SVector2, PlusOperator)
	{
		//�������m�̑����Z
		SVector2 v1(1, 2);
		SVector2 v2(3, 4);
		SVector2 v3 = v1 + v2;
		EXPECT_FLOAT_EQ(v3.x, 4);
		EXPECT_FLOAT_EQ(v3.y, 6);

		//�����Ƃ̑����Z
		SVector2 v4 = v1 + 1;
		EXPECT_FLOAT_EQ(v4.x, 2);
		EXPECT_FLOAT_EQ(v4.y, 3);

		//���������_�����m�̑����Z
		SVector2 v5(0.4f, 0.9f);
		SVector2 v6(11.7f, 8.3f);
		SVector2 v7 = v5 + v6;
		EXPECT_FLOAT_EQ(v7.x, 12.1f);
		EXPECT_FLOAT_EQ(v7.y, 9.2f);

		//���������_���Ƃ̑����Z
		SVector2 v8 = v5 + 1.1f;
		EXPECT_FLOAT_EQ(v8.x, 1.5f);
		EXPECT_FLOAT_EQ(v8.y, 2.0f);

		//���Ԃ��t�ł������l�ɂȂ�
		SVector2 v9 = v1 + v2;
		SVector2 v10 = v2 + v1;
		EXPECT_FLOAT_EQ(v9.x, v10.x);
		EXPECT_FLOAT_EQ(v9.y, v10.y);

		//�����@��
		SVector2 v11 = v1 + (v2 + v3);
		SVector2 v12 = (v1 + v2) + v3;
		EXPECT_FLOAT_EQ(v11.x, v12.x);
		EXPECT_FLOAT_EQ(v11.y, v12.y);
	}

	TEST(SVector2, MinusOperator)
	{
		//�������m�̈����Z
		SVector2 v1(1, 2);
		SVector2 v2(3, 4);
		SVector2 v3 = v1 - v2;
		EXPECT_FLOAT_EQ(v3.x, -2);
		EXPECT_FLOAT_EQ(v3.y, -2);

		//�����Ƃ̈����Z
		SVector2 v4 = v1 - 1;
		EXPECT_FLOAT_EQ(v4.x, 0);
		EXPECT_FLOAT_EQ(v4.y, 1);

		//���������_�����m�̈����Z
		SVector2 v5(0.4f, 0.9f);
		SVector2 v6(11.7f, 8.3f);
		SVector2 v7 = v5 - v6;
		EXPECT_FLOAT_EQ(v7.x, -11.3f);
		EXPECT_FLOAT_EQ(v7.y, -7.4f);

		//���������_���Ƃ̈����Z
		SVector2 v8 = v5 - 1.1f;
		EXPECT_FLOAT_EQ(v8.x, -0.7f);
		EXPECT_FLOAT_EQ(v8.y, -0.2f);

		//���Ԃ��t�ł������l�ɂȂ�
		SVector2 v9 = v1 - v2;
		SVector2 v10 = v2 - v1;
		EXPECT_FLOAT_EQ(v9.x, -v10.x);
		EXPECT_FLOAT_EQ(v9.y, -v10.y);

		//���z�@��
		SVector2 v11 = v1 - (v2 - v3);
		SVector2 v12 = v1 - v2 + v3;
		EXPECT_FLOAT_EQ(v11.x, v12.x);
		EXPECT_FLOAT_EQ(v11.y, v12.y);
	}

	TEST(SVector2, MultipleOperator)
	{
		//�����Ƃ̊|���Z
		SVector2 v1(1, 2);
		SVector2 v2 = v1 * 2;
		EXPECT_FLOAT_EQ(v2.x, 2);
		EXPECT_FLOAT_EQ(v2.y, 4);

		//���������_���Ƃ̊|���Z
		SVector2 v3 = v1 * 0.5f;
		EXPECT_FLOAT_EQ(v3.x, 0.5f);
		EXPECT_FLOAT_EQ(v3.y, 1.0f);

		//���Ԃ��t�ł������l�ɂȂ�
		SVector2 v4 = v1 * 2;
		SVector2 v5 = 2 * v1;
		EXPECT_FLOAT_EQ(v4.x, v5.x);
		EXPECT_FLOAT_EQ(v4.y, v5.y);

		//���̐��Ƃ̊|���Z
		SVector2 v6 = v1 * -2;
		EXPECT_FLOAT_EQ(v6.x, -2);
		EXPECT_FLOAT_EQ(v6.y, -4);

		//���̕��������_���Ƃ̊|���Z
		SVector2 v7 = v1 * -0.5f;
		EXPECT_FLOAT_EQ(v7.x, -0.5f);
		EXPECT_FLOAT_EQ(v7.y, -1.0f);
	}

	TEST(SVector2, DivisionOperator)
	{
		//�����Ƃ̊���Z
		SVector2 v1(1, 2);
		SVector2 v2 = v1 / 2;
		EXPECT_FLOAT_EQ(v2.x, 0.5f);
		EXPECT_FLOAT_EQ(v2.y, 1);

		//���������_���Ƃ̊���Z
		SVector2 v3 = v1 / 0.5f;
		EXPECT_FLOAT_EQ(v3.x, 2);
		EXPECT_FLOAT_EQ(v3.y, 4);

		//���̐��Ƃ̊���Z
		SVector2 v4 = v1 / -2;
		EXPECT_FLOAT_EQ(v4.x, -0.5f);
		EXPECT_FLOAT_EQ(v4.y, -1);

		//���̕��������_���Ƃ̊���Z
		SVector2 v5 = v1 / -0.5f;
		EXPECT_FLOAT_EQ(v5.x, -2);
		EXPECT_FLOAT_EQ(v5.y, -4);
	}
}