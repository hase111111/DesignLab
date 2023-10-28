#include "pch.h"

#include "../DesignLab/designlab_polygon2.h"


namespace designlab::test::common::math
{
	TEST(Polygon2Test, GetMaxXTest)
	{
		//�O�p�`�𐶐����āCX���W�̍ő�l�����߂�
		designlab::Polygon2 polygon;
		polygon.AddVertex({ 0, 0 });
		polygon.AddVertex({ 1, 0 });
		polygon.AddVertex({ 0, 1 });

		EXPECT_FLOAT_EQ(polygon.GetMaxX(), 1.0f);

		//�l�p�`�𐶐����āCX���W�̍ő�l�����߂�
		designlab::Polygon2 polygon2;
		polygon2.AddVertex({ 0, 0 });
		polygon2.AddVertex({ 1, 0 });
		polygon2.AddVertex({ 1, 1 });
		polygon2.AddVertex({ 0, 1 });

		EXPECT_FLOAT_EQ(polygon2.GetMaxX(), 1.0f);

		//��قǂ�4�p�`�ɓ_��ǉ����āCX���W�̍ő�l�����߂�
		polygon2.AddVertex({ 2, 0.5 });
		polygon2.AddVertex({ -5, 0.5 });

		EXPECT_FLOAT_EQ(polygon2.GetMaxX(), 2.0f);

		//6�p�`�𐶐����āCX���W�̍ő�l�����߂�
		designlab::Polygon2 polygon3;
		polygon3.AddVertex({ 0, 0 });
		polygon3.AddVertex({ 1, 0 });
		polygon3.AddVertex({ 1, 1 });
		polygon3.AddVertex({ 0, 1 });
		polygon3.AddVertex({ -1, 1 });
		polygon3.AddVertex({ -1, 0 });

		EXPECT_FLOAT_EQ(polygon3.GetMaxX(), 1.0f);

		//��قǂ�6�p�`����_���폜���āCX���W�̍ő�l�����߂�
		polygon3.RemoveLastVertex();
		polygon3.RemoveLastVertex();

		EXPECT_FLOAT_EQ(polygon3.GetMaxX(), 1.0f);
	}

	//GetMinX�̃e�X�g
	TEST(Polygon2Test, GetMinXTest)
	{
		//�O�p�`�𐶐����āCX���W�̍ŏ��l�����߂�
		designlab::Polygon2 polygon;
		polygon.AddVertex({ 0, 0 });
		polygon.AddVertex({ 1, 0 });
		polygon.AddVertex({ 0, 1 });

		EXPECT_FLOAT_EQ(polygon.GetMinX(), 0.0f);

		//�l�p�`�𐶐����āCX���W�̍ŏ��l�����߂�
		designlab::Polygon2 polygon2;
		polygon2.AddVertex({ 0, 0 });
		polygon2.AddVertex({ 1, 0 });
		polygon2.AddVertex({ 1, 1 });
		polygon2.AddVertex({ 0, 1 });

		EXPECT_FLOAT_EQ(polygon2.GetMinX(), 0.0f);

		//��قǂ�4�p�`�ɓ_��ǉ����āCX���W�̍ŏ��l�����߂�
		polygon2.AddVertex({ 2, 0.5 });
		polygon2.AddVertex({ -5, 0.5 });

		EXPECT_FLOAT_EQ(polygon2.GetMinX(), -5.0f);

		//6�p�`�𐶐����āCX���W�̍ŏ��l�����߂�
		designlab::Polygon2 polygon3;
		polygon3.AddVertex({ 0, 0 });
		polygon3.AddVertex({ 1, 0 });
		polygon3.AddVertex({ 1, 1 });
		polygon3.AddVertex({ 0, 1 });
		polygon3.AddVertex({ -1, 1 });
		polygon3.AddVertex({ -1, 0 });

		EXPECT_FLOAT_EQ(polygon3.GetMinX(), -1.0f);

		//��قǂ�6�p�`����_���폜���āCX���W�̍ŏ��l�����߂�
		polygon3.RemoveLastVertex();
		polygon3.RemoveLastVertex();

		EXPECT_FLOAT_EQ(polygon3.GetMinX(), 0.0f);
	}

	//GetMaxY�̃e�X�g
	TEST(Polygon2Test, GetMaxYTest)
	{
		//�O�p�`�𐶐����āCY���W�̍ő�l�����߂�
		designlab::Polygon2 polygon;
		polygon.AddVertex({ 0, 0 });
		polygon.AddVertex({ 1, 0 });
		polygon.AddVertex({ 0, 1 });

		EXPECT_FLOAT_EQ(polygon.GetMaxY(), 1.0f);

		//�l�p�`�𐶐����āCY���W�̍ő�l�����߂�
		designlab::Polygon2 polygon2;
		polygon2.AddVertex({ 0, 0 });
		polygon2.AddVertex({ 1, 0 });
		polygon2.AddVertex({ 1, 1 });
		polygon2.AddVertex({ 0, 1 });

		EXPECT_FLOAT_EQ(polygon2.GetMaxY(), 1.0f);

		//��قǂ�4�p�`�ɓ_��ǉ����āCY���W�̍ő�l�����߂�
		polygon2.AddVertex({ 2, 0.5 });
		polygon2.AddVertex({ -5, 0.5 });

		EXPECT_FLOAT_EQ(polygon2.GetMaxY(), 1.0f);

		//6�p�`�𐶐����āCY���W�̍ő�l�����߂�
		designlab::Polygon2 polygon3;
		polygon3.AddVertex({ 0, 0 });
		polygon3.AddVertex({ 1, 0 });
		polygon3.AddVertex({ 1, 1 });
		polygon3.AddVertex({ 0, 1 });
		polygon3.AddVertex({ -1, 1 });
		polygon3.AddVertex({ -1, 0 });

		EXPECT_FLOAT_EQ(polygon3.GetMaxY(), 1.0f);

		//��قǂ�6�p�`����_���폜���āCY���W�̍ő�l�����߂�
		polygon3.RemoveLastVertex();
		polygon3.RemoveLastVertex();

		EXPECT_FLOAT_EQ(polygon3.GetMaxY(), 1.0f);
	}

	TEST(Polygon2Test, GetMinYTest)
	{
		//�O�p�`�𐶐����āCY���W�̍ŏ��l�����߂�
		designlab::Polygon2 polygon;
		polygon.AddVertex({ 0, 0 });
		polygon.AddVertex({ 1, 0 });
		polygon.AddVertex({ 0, 1 });

		EXPECT_FLOAT_EQ(polygon.GetMinY(), 0.0f);

		//�l�p�`�𐶐����āCY���W�̍ŏ��l�����߂�
		designlab::Polygon2 polygon2;
		polygon2.AddVertex({ 0, 0 });
		polygon2.AddVertex({ 1, 0 });
		polygon2.AddVertex({ 1, 1 });
		polygon2.AddVertex({ 0, 1 });

		EXPECT_FLOAT_EQ(polygon2.GetMinY(), 0.0f);

		//��قǂ�4�p�`�ɓ_��ǉ����āCY���W�̍ŏ��l�����߂�
		polygon2.AddVertex({ 2, 0.5 });
		polygon2.AddVertex({ -5, 0.5 });

		EXPECT_FLOAT_EQ(polygon2.GetMinY(), 0.0f);

		//6�p�`�𐶐����āCY���W�̍ŏ��l�����߂�
		designlab::Polygon2 polygon3;
		polygon3.AddVertex({ 0, 0 });
		polygon3.AddVertex({ 1, 0 });
		polygon3.AddVertex({ 1, 1 });
		polygon3.AddVertex({ 0, 1 });
		polygon3.AddVertex({ -1, 1 });
		polygon3.AddVertex({ -1, 0 });

		EXPECT_FLOAT_EQ(polygon3.GetMinY(), 0.0f);

		//��قǂ�6�p�`����_���폜���āCY���W�̍ŏ��l�����߂�
		polygon3.RemoveLastVertex();
		polygon3.RemoveLastVertex();

		EXPECT_FLOAT_EQ(polygon3.GetMinY(), 0.0f);
	}

}	//namespace dl_vec_test