#include "pch.h"

#include "../DesignLab/designlab_polygon.h"


namespace dl_vec_test
{
	//isInside�֐��̃e�X�g(�E���)
	TEST(SPolygon2, IsInsideRightTurn)
	{
		//4�p�`���쐬���C�����ɂ���_�ƊO���ɂ���_���m�F����
		designlab::SPolygon2 polygon;
		polygon.addVertex({ 0, 0 });
		polygon.addVertex({ 1, 0 });
		polygon.addVertex({ 1, 1 });
		polygon.addVertex({ 0, 1 });

		EXPECT_TRUE(polygon.isInside({ 0.5f, 0.5f }));		//�����ɂ���_
		EXPECT_TRUE(polygon.isInside({ 0.3f, 0.7f }));
		EXPECT_TRUE(polygon.isInside({ 0.25f, 0.45f }));
		EXPECT_TRUE(polygon.isInside({ 0.8f, 0.95f }));

		EXPECT_TRUE(polygon.isInside({ 0.5f, 0.0f }));		//�ӂ̏�ɂ���_
		EXPECT_TRUE(polygon.isInside({ 1.0f, 0.5f }));
		EXPECT_TRUE(polygon.isInside({ 0.5f, 1.0f }));
		EXPECT_TRUE(polygon.isInside({ 0.0f, 0.5f }));

		EXPECT_TRUE(polygon.isInside({ 0.0f, 0.0f }));		//���_�ɂ���_
		EXPECT_TRUE(polygon.isInside({ 1.0f, 0.0f }));
		EXPECT_TRUE(polygon.isInside({ 1.0f, 1.0f }));
		EXPECT_TRUE(polygon.isInside({ 0.0f, 1.0f }));

		EXPECT_FALSE(polygon.isInside({ 1.5f, 1.5f }));		//�O���ɂ���_
		EXPECT_FALSE(polygon.isInside({ -0.5f, 1.5f }));
		EXPECT_FALSE(polygon.isInside({ 1.5f, -0.5f }));
		EXPECT_FALSE(polygon.isInside({ -0.5f, -0.5f }));

		EXPECT_FALSE(polygon.isInside({ 0.0f, -0.5f }));	//�ӂ̉�������ɂ���_
		EXPECT_FALSE(polygon.isInside({ 0.0f, 1.5f }));

		EXPECT_FALSE(polygon.isInside({ 10000.0f, 10000.0f }));	//���Ȃ�O���ɂ���_
		EXPECT_FALSE(polygon.isInside({ -10000.0f, 10000.0f }));
		EXPECT_FALSE(polygon.isInside({ 10000.0f, -10000.0f }));
		EXPECT_FALSE(polygon.isInside({ -10000.0f, -10000.0f }));


		//6�p�`���쐬���C�����ɂ���_�ƊO���ɂ���_���m�F����
		designlab::SPolygon2 polygon2;
		polygon2.addVertex({ 0, -0.5f });
		polygon2.addVertex({ 1, 0 });
		polygon2.addVertex({ 1, 1 });
		polygon2.addVertex({ 0, 1.5f });
		polygon2.addVertex({ -1, 1 });
		polygon2.addVertex({ -1, 0 });

		EXPECT_TRUE(polygon2.isInside({ 0.5f, 0.5f }));
		EXPECT_TRUE(polygon2.isInside({ 0.3f, 0.7f }));
		EXPECT_TRUE(polygon2.isInside({ 0.25f, 0.45f }));
		EXPECT_TRUE(polygon2.isInside({ 0.8f, 0.95f }));
		EXPECT_FALSE(polygon2.isInside({ 1.5f, 0.5f }));

		//���_��2�ȉ��̑��p�`�ł͕K�����s����D
		designlab::SPolygon2 polygon3;
		polygon3.addVertex({ 0, 0 });
		polygon3.addVertex({ 1, 0 });

		EXPECT_FALSE(polygon3.isInside({ 0.5f, 0.5f }));

	}

	//isInside�֐��̃e�X�g(�����)
	TEST(SPolygon2, IsInsideLeftTurn)
	{
		//4�p�`���쐬���C�����ɂ���_�ƊO���ɂ���_���m�F����
		designlab::SPolygon2 polygon;
		polygon.addVertex({ 0, 0 });
		polygon.addVertex({ 0, 1 });
		polygon.addVertex({ 1, 1 });
		polygon.addVertex({ 1, 0 });

		EXPECT_TRUE(polygon.isInside({ 0.5f, 0.5f }));		//�����ɂ���_
		EXPECT_TRUE(polygon.isInside({ 0.3f, 0.7f }));
		EXPECT_TRUE(polygon.isInside({ 0.25f, 0.45f }));
		EXPECT_TRUE(polygon.isInside({ 0.8f, 0.95f }));

		EXPECT_TRUE(polygon.isInside({ 0.5f, 0.0f }));		//�ӂ̏�ɂ���_
		EXPECT_TRUE(polygon.isInside({ 1.0f, 0.5f }));
		EXPECT_TRUE(polygon.isInside({ 0.5f, 1.0f }));
		EXPECT_TRUE(polygon.isInside({ 0.0f, 0.5f }));

		EXPECT_TRUE(polygon.isInside({ 0.0f, 0.0f }));		//���_�ɂ���_
		EXPECT_TRUE(polygon.isInside({ 1.0f, 0.0f }));
		EXPECT_TRUE(polygon.isInside({ 1.0f, 1.0f }));
		EXPECT_TRUE(polygon.isInside({ 0.0f, 1.0f }));

		EXPECT_FALSE(polygon.isInside({ 1.5f, 1.5f }));		//�O���ɂ���_
		EXPECT_FALSE(polygon.isInside({ -0.5f, 1.5f }));
		EXPECT_FALSE(polygon.isInside({ 1.5f, -0.5f }));
		EXPECT_FALSE(polygon.isInside({ -0.5f, -0.5f }));

		EXPECT_FALSE(polygon.isInside({ 0.0f, -0.5f }));	//�ӂ̉�������ɂ���_
		EXPECT_FALSE(polygon.isInside({ 0.0f, 1.5f }));

		EXPECT_FALSE(polygon.isInside({ 10000.0f, 100000.0f }));
		EXPECT_FALSE(polygon.isInside({ -10000.0f, 10000.0f }));
		EXPECT_FALSE(polygon.isInside({ 10000.0f, -10000.0f }));
		EXPECT_FALSE(polygon.isInside({ -10000.0f, -10000.0f }));


		//6�p�`���쐬���C�����ɂ���_�ƊO���ɂ���_���m�F����
		designlab::SPolygon2 polygon2;
		polygon2.addVertex({ 0, -0.5f });
		polygon2.addVertex({ -1, 0 });
		polygon2.addVertex({ -1, 1 });
		polygon2.addVertex({ 0, 1.5f });
		polygon2.addVertex({ 1, 1 });
		polygon2.addVertex({ 1, 0 });

		EXPECT_TRUE(polygon2.isInside({ 0.5f, 0.5f }));
		EXPECT_TRUE(polygon2.isInside({ 0.3f, 0.7f }));
		EXPECT_TRUE(polygon2.isInside({ 0.25f, 0.45f }));
		EXPECT_TRUE(polygon2.isInside({ 0.8f, 0.95f }));
		EXPECT_FALSE(polygon2.isInside({ 1.5f, 0.5f }));

		//���_��2�ȉ��̑��p�`�ł͕K�����s����D
		designlab::SPolygon2 polygon3;
		polygon3.addVertex({ 0, 0 });
		polygon3.addVertex({ 1, 0 });

		EXPECT_FALSE(polygon3.isInside({ 0.5f, 0.5f }));

	}
}