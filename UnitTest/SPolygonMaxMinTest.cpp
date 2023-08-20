#include "pch.h"
#include "../DesignLab/MyPolygon.h"


namespace SPolygon2Test
{
	//GetMaxX�֐��̃e�X�g
	TEST(SPolygon2Func, GetMaxX)
	{
		//�O�p�`�𐶐����āCX���W�̍ő�l�����߂�
		dl_vec::SPolygon2 polygon;
		polygon.addVertex({ 0, 0 });
		polygon.addVertex({ 1, 0 });
		polygon.addVertex({ 0, 1 });

		EXPECT_FLOAT_EQ(polygon.getMaxX(), 1.0f);

		//�l�p�`�𐶐����āCX���W�̍ő�l�����߂�
		dl_vec::SPolygon2 polygon2;
		polygon2.addVertex({ 0, 0 });
		polygon2.addVertex({ 1, 0 });
		polygon2.addVertex({ 1, 1 });
		polygon2.addVertex({ 0, 1 });

		EXPECT_FLOAT_EQ(polygon2.getMaxX(), 1.0f);

		//��قǂ�4�p�`�ɓ_��ǉ����āCX���W�̍ő�l�����߂�
		polygon2.addVertex({ 2, 0.5 });
		polygon2.addVertex({ -5, 0.5 });

		EXPECT_FLOAT_EQ(polygon2.getMaxX(), 2.0f);

		//6�p�`�𐶐����āCX���W�̍ő�l�����߂�
		dl_vec::SPolygon2 polygon3;
		polygon3.addVertex({ 0, 0 });
		polygon3.addVertex({ 1, 0 });
		polygon3.addVertex({ 1, 1 });
		polygon3.addVertex({ 0, 1 });
		polygon3.addVertex({ -1, 1 });
		polygon3.addVertex({ -1, 0 });

		EXPECT_FLOAT_EQ(polygon3.getMaxX(), 1.0f);

		//��قǂ�6�p�`����_���폜���āCX���W�̍ő�l�����߂�
		polygon3.removeLastVertex();
		polygon3.removeLastVertex();

		EXPECT_FLOAT_EQ(polygon3.getMaxX(), 1.0f);
	}

	//GetMinX�̃e�X�g
	TEST(SPolygon2Func, GetMinX)
	{
		//�O�p�`�𐶐����āCX���W�̍ŏ��l�����߂�
		dl_vec::SPolygon2 polygon;
		polygon.addVertex({ 0, 0 });
		polygon.addVertex({ 1, 0 });
		polygon.addVertex({ 0, 1 });

		EXPECT_FLOAT_EQ(polygon.getMinX(), 0.0f);

		//�l�p�`�𐶐����āCX���W�̍ŏ��l�����߂�
		dl_vec::SPolygon2 polygon2;
		polygon2.addVertex({ 0, 0 });
		polygon2.addVertex({ 1, 0 });
		polygon2.addVertex({ 1, 1 });
		polygon2.addVertex({ 0, 1 });

		EXPECT_FLOAT_EQ(polygon2.getMinX(), 0.0f);

		//��قǂ�4�p�`�ɓ_��ǉ����āCX���W�̍ŏ��l�����߂�
		polygon2.addVertex({ 2, 0.5 });
		polygon2.addVertex({ -5, 0.5 });

		EXPECT_FLOAT_EQ(polygon2.getMinX(), -5.0f);

		//6�p�`�𐶐����āCX���W�̍ŏ��l�����߂�
		dl_vec::SPolygon2 polygon3;
		polygon3.addVertex({ 0, 0 });
		polygon3.addVertex({ 1, 0 });
		polygon3.addVertex({ 1, 1 });
		polygon3.addVertex({ 0, 1 });
		polygon3.addVertex({ -1, 1 });
		polygon3.addVertex({ -1, 0 });

		EXPECT_FLOAT_EQ(polygon3.getMinX(), -1.0f);

		//��قǂ�6�p�`����_���폜���āCX���W�̍ŏ��l�����߂�
		polygon3.removeLastVertex();
		polygon3.removeLastVertex();

		EXPECT_FLOAT_EQ(polygon3.getMinX(), 0.0f);
	}

	//GetMaxY�̃e�X�g
	TEST(SPolygon2Func, GetMaxY)
	{
		//�O�p�`�𐶐����āCY���W�̍ő�l�����߂�
		dl_vec::SPolygon2 polygon;
		polygon.addVertex({ 0, 0 });
		polygon.addVertex({ 1, 0 });
		polygon.addVertex({ 0, 1 });

		EXPECT_FLOAT_EQ(polygon.getMaxY(), 1.0f);

		//�l�p�`�𐶐����āCY���W�̍ő�l�����߂�
		dl_vec::SPolygon2 polygon2;
		polygon2.addVertex({ 0, 0 });
		polygon2.addVertex({ 1, 0 });
		polygon2.addVertex({ 1, 1 });
		polygon2.addVertex({ 0, 1 });

		EXPECT_FLOAT_EQ(polygon2.getMaxY(), 1.0f);

		//��قǂ�4�p�`�ɓ_��ǉ����āCY���W�̍ő�l�����߂�
		polygon2.addVertex({ 2, 0.5 });
		polygon2.addVertex({ -5, 0.5 });

		EXPECT_FLOAT_EQ(polygon2.getMaxY(), 1.0f);

		//6�p�`�𐶐����āCY���W�̍ő�l�����߂�
		dl_vec::SPolygon2 polygon3;
		polygon3.addVertex({ 0, 0 });
		polygon3.addVertex({ 1, 0 });
		polygon3.addVertex({ 1, 1 });
		polygon3.addVertex({ 0, 1 });
		polygon3.addVertex({ -1, 1 });
		polygon3.addVertex({ -1, 0 });

		EXPECT_FLOAT_EQ(polygon3.getMaxY(), 1.0f);

		//��قǂ�6�p�`����_���폜���āCY���W�̍ő�l�����߂�
		polygon3.removeLastVertex();
		polygon3.removeLastVertex();

		EXPECT_FLOAT_EQ(polygon3.getMaxY(), 1.0f);
	}

	TEST(SPolygon2Func, GetMinY)
	{
		//�O�p�`�𐶐����āCY���W�̍ŏ��l�����߂�
		dl_vec::SPolygon2 polygon;
		polygon.addVertex({ 0, 0 });
		polygon.addVertex({ 1, 0 });
		polygon.addVertex({ 0, 1 });

		EXPECT_FLOAT_EQ(polygon.getMinY(), 0.0f);

		//�l�p�`�𐶐����āCY���W�̍ŏ��l�����߂�
		dl_vec::SPolygon2 polygon2;
		polygon2.addVertex({ 0, 0 });
		polygon2.addVertex({ 1, 0 });
		polygon2.addVertex({ 1, 1 });
		polygon2.addVertex({ 0, 1 });

		EXPECT_FLOAT_EQ(polygon2.getMinY(), 0.0f);

		//��قǂ�4�p�`�ɓ_��ǉ����āCY���W�̍ŏ��l�����߂�
		polygon2.addVertex({ 2, 0.5 });
		polygon2.addVertex({ -5, 0.5 });

		EXPECT_FLOAT_EQ(polygon2.getMinY(), 0.0f);

		//6�p�`�𐶐����āCY���W�̍ŏ��l�����߂�
		dl_vec::SPolygon2 polygon3;
		polygon3.addVertex({ 0, 0 });
		polygon3.addVertex({ 1, 0 });
		polygon3.addVertex({ 1, 1 });
		polygon3.addVertex({ 0, 1 });
		polygon3.addVertex({ -1, 1 });
		polygon3.addVertex({ -1, 0 });

		EXPECT_FLOAT_EQ(polygon3.getMinY(), 0.0f);

		//��قǂ�6�p�`����_���폜���āCY���W�̍ŏ��l�����߂�
		polygon3.removeLastVertex();
		polygon3.removeLastVertex();

		EXPECT_FLOAT_EQ(polygon3.getMinY(), 0.0f);
	}

}	//namespace SPolygon2Test