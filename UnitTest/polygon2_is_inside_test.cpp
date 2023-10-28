#include "pch.h"

#include "../DesignLab/designlab_polygon2.h"


namespace designlab::test::common::math
{
	//IsInside�֐��̃e�X�g(�E���)
	TEST(Polygon2Test, IsInsideRightTurn)
	{
		//4�p�`���쐬���C�����ɂ���_�ƊO���ɂ���_���m�F����
		designlab::Polygon2 polygon;
		polygon.AddVertex({ 0, 0 });
		polygon.AddVertex({ 1, 0 });
		polygon.AddVertex({ 1, 1 });
		polygon.AddVertex({ 0, 1 });

		EXPECT_TRUE(polygon.IsInside({ 0.5f, 0.5f }));		//�����ɂ���_
		EXPECT_TRUE(polygon.IsInside({ 0.3f, 0.7f }));
		EXPECT_TRUE(polygon.IsInside({ 0.25f, 0.45f }));
		EXPECT_TRUE(polygon.IsInside({ 0.8f, 0.95f }));

		EXPECT_TRUE(polygon.IsInside({ 0.5f, 0.0f }));		//�ӂ̏�ɂ���_
		EXPECT_TRUE(polygon.IsInside({ 1.0f, 0.5f }));
		EXPECT_TRUE(polygon.IsInside({ 0.5f, 1.0f }));
		EXPECT_TRUE(polygon.IsInside({ 0.0f, 0.5f }));

		EXPECT_TRUE(polygon.IsInside({ 0.0f, 0.0f }));		//���_�ɂ���_
		EXPECT_TRUE(polygon.IsInside({ 1.0f, 0.0f }));
		EXPECT_TRUE(polygon.IsInside({ 1.0f, 1.0f }));
		EXPECT_TRUE(polygon.IsInside({ 0.0f, 1.0f }));

		EXPECT_FALSE(polygon.IsInside({ 1.5f, 1.5f }));		//�O���ɂ���_
		EXPECT_FALSE(polygon.IsInside({ -0.5f, 1.5f }));
		EXPECT_FALSE(polygon.IsInside({ 1.5f, -0.5f }));
		EXPECT_FALSE(polygon.IsInside({ -0.5f, -0.5f }));

		EXPECT_FALSE(polygon.IsInside({ 0.0f, -0.5f }));	//�ӂ̉�������ɂ���_
		EXPECT_FALSE(polygon.IsInside({ 0.0f, 1.5f }));

		EXPECT_FALSE(polygon.IsInside({ 10000.0f, 10000.0f }));	//���Ȃ�O���ɂ���_
		EXPECT_FALSE(polygon.IsInside({ -10000.0f, 10000.0f }));
		EXPECT_FALSE(polygon.IsInside({ 10000.0f, -10000.0f }));
		EXPECT_FALSE(polygon.IsInside({ -10000.0f, -10000.0f }));


		//6�p�`���쐬���C�����ɂ���_�ƊO���ɂ���_���m�F����
		designlab::Polygon2 polygon2;
		polygon2.AddVertex({ 0, -0.5f });
		polygon2.AddVertex({ 1, 0 });
		polygon2.AddVertex({ 1, 1 });
		polygon2.AddVertex({ 0, 1.5f });
		polygon2.AddVertex({ -1, 1 });
		polygon2.AddVertex({ -1, 0 });

		EXPECT_TRUE(polygon2.IsInside({ 0.5f, 0.5f }));
		EXPECT_TRUE(polygon2.IsInside({ 0.3f, 0.7f }));
		EXPECT_TRUE(polygon2.IsInside({ 0.25f, 0.45f }));
		EXPECT_TRUE(polygon2.IsInside({ 0.8f, 0.95f }));
		EXPECT_FALSE(polygon2.IsInside({ 1.5f, 0.5f }));

		//���_��2�ȉ��̑��p�`�ł͕K�����s����D
		designlab::Polygon2 polygon3;
		polygon3.AddVertex({ 0, 0 });
		polygon3.AddVertex({ 1, 0 });

		EXPECT_FALSE(polygon3.IsInside({ 0.5f, 0.5f }));

	}

	//IsInside�֐��̃e�X�g(�����)
	TEST(Polygon2Test, IsInsideLeftTurn)
	{
		//4�p�`���쐬���C�����ɂ���_�ƊO���ɂ���_���m�F����
		designlab::Polygon2 polygon;
		polygon.AddVertex({ 0, 0 });
		polygon.AddVertex({ 0, 1 });
		polygon.AddVertex({ 1, 1 });
		polygon.AddVertex({ 1, 0 });

		EXPECT_TRUE(polygon.IsInside({ 0.5f, 0.5f }));		//�����ɂ���_
		EXPECT_TRUE(polygon.IsInside({ 0.3f, 0.7f }));
		EXPECT_TRUE(polygon.IsInside({ 0.25f, 0.45f }));
		EXPECT_TRUE(polygon.IsInside({ 0.8f, 0.95f }));

		EXPECT_TRUE(polygon.IsInside({ 0.5f, 0.0f }));		//�ӂ̏�ɂ���_
		EXPECT_TRUE(polygon.IsInside({ 1.0f, 0.5f }));
		EXPECT_TRUE(polygon.IsInside({ 0.5f, 1.0f }));
		EXPECT_TRUE(polygon.IsInside({ 0.0f, 0.5f }));

		EXPECT_TRUE(polygon.IsInside({ 0.0f, 0.0f }));		//���_�ɂ���_
		EXPECT_TRUE(polygon.IsInside({ 1.0f, 0.0f }));
		EXPECT_TRUE(polygon.IsInside({ 1.0f, 1.0f }));
		EXPECT_TRUE(polygon.IsInside({ 0.0f, 1.0f }));

		EXPECT_FALSE(polygon.IsInside({ 1.5f, 1.5f }));		//�O���ɂ���_
		EXPECT_FALSE(polygon.IsInside({ -0.5f, 1.5f }));
		EXPECT_FALSE(polygon.IsInside({ 1.5f, -0.5f }));
		EXPECT_FALSE(polygon.IsInside({ -0.5f, -0.5f }));

		EXPECT_FALSE(polygon.IsInside({ 0.0f, -0.5f }));	//�ӂ̉�������ɂ���_
		EXPECT_FALSE(polygon.IsInside({ 0.0f, 1.5f }));

		EXPECT_FALSE(polygon.IsInside({ 10000.0f, 100000.0f }));
		EXPECT_FALSE(polygon.IsInside({ -10000.0f, 10000.0f }));
		EXPECT_FALSE(polygon.IsInside({ 10000.0f, -10000.0f }));
		EXPECT_FALSE(polygon.IsInside({ -10000.0f, -10000.0f }));


		//6�p�`���쐬���C�����ɂ���_�ƊO���ɂ���_���m�F����
		designlab::Polygon2 polygon2;
		polygon2.AddVertex({ 0, -0.5f });
		polygon2.AddVertex({ -1, 0 });
		polygon2.AddVertex({ -1, 1 });
		polygon2.AddVertex({ 0, 1.5f });
		polygon2.AddVertex({ 1, 1 });
		polygon2.AddVertex({ 1, 0 });

		EXPECT_TRUE(polygon2.IsInside({ 0.5f, 0.5f }));
		EXPECT_TRUE(polygon2.IsInside({ 0.3f, 0.7f }));
		EXPECT_TRUE(polygon2.IsInside({ 0.25f, 0.45f }));
		EXPECT_TRUE(polygon2.IsInside({ 0.8f, 0.95f }));
		EXPECT_FALSE(polygon2.IsInside({ 1.5f, 0.5f }));

		//���_��2�ȉ��̑��p�`�ł͕K�����s����D
		designlab::Polygon2 polygon3;
		polygon3.AddVertex({ 0, 0 });
		polygon3.AddVertex({ 1, 0 });

		EXPECT_FALSE(polygon3.IsInside({ 0.5f, 0.5f }));

	}
}