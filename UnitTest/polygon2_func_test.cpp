#include "pch.h"

#include "../DesignLab/designlab_polygon2.h"
#include "../DesignLab/designlab_polygon.cpp"


namespace dl_vec_test
{
	//addVertex�֐��̃e�X�g
	TEST(Polygon2, AddVertex)
	{
		//�l�p�`���쐬���C���̒��_�̍��W���m�F����
		designlab::Polygon2 polygon;
		polygon.AddVertex({ 0, 0 });
		polygon.AddVertex({ 1, 0 });
		polygon.AddVertex({ 1, 1 });
		polygon.AddVertex({ 0, 1 });

		EXPECT_EQ(polygon.GetVertexNum(), 4);
		EXPECT_EQ(polygon.GetVertex(0), designlab::Vector2(0, 0));
		EXPECT_EQ(polygon.GetVertex(1), designlab::Vector2(1, 0));
		EXPECT_EQ(polygon.GetVertex(2), designlab::Vector2(1, 1));
		EXPECT_EQ(polygon.GetVertex(3), designlab::Vector2(0, 1));

		//6�p�`���쐬���C���̒��_�̍��W���m�F����
		designlab::Polygon2 polygon2;
		polygon2.AddVertex({ 0, 0 });
		polygon2.AddVertex({ 1, 0 });
		polygon2.AddVertex({ 1, 1 });
		polygon2.AddVertex({ 0, 1 });
		polygon2.AddVertex({ -1, 1 });
		polygon2.AddVertex({ -1, 0 });

		EXPECT_EQ(polygon2.GetVertexNum(), 6);
		EXPECT_EQ(polygon2.GetVertex(0), designlab::Vector2(0, 0));
		EXPECT_EQ(polygon2.GetVertex(1), designlab::Vector2(1, 0));
		EXPECT_EQ(polygon2.GetVertex(2), designlab::Vector2(1, 1));
		EXPECT_EQ(polygon2.GetVertex(3), designlab::Vector2(0, 1));
		EXPECT_EQ(polygon2.GetVertex(4), designlab::Vector2(-1, 1));
		EXPECT_EQ(polygon2.GetVertex(5), designlab::Vector2(-1, 0));
	}

	//addVertexCheckForDuplicates�֐��̃e�X�g
	TEST(Polygon2, AddVertexCheckForDuplicates)
	{
		//�d�����钸�_��ǉ����C���̒��_�̍��W���m�F����
		designlab::Polygon2 polygon;
		polygon.AddVertexCheckForDuplicates({ 0, 0 });
		polygon.AddVertexCheckForDuplicates({ 1, 0 });
		polygon.AddVertexCheckForDuplicates({ 1, 1 });
		polygon.AddVertexCheckForDuplicates({ 0, 1 });
		polygon.AddVertexCheckForDuplicates({ 0, 0 });	//�d�����钸�_

		EXPECT_EQ(polygon.GetVertexNum(), 4);
		EXPECT_EQ(polygon.GetVertex(0), designlab::Vector2(0, 0));
		EXPECT_EQ(polygon.GetVertex(1), designlab::Vector2(1, 0));
		EXPECT_EQ(polygon.GetVertex(2), designlab::Vector2(1, 1));
		EXPECT_EQ(polygon.GetVertex(3), designlab::Vector2(0, 1));

		//�d�����Ȃ����_��ǉ����C���̒��_�̍��W���m�F����
		designlab::Polygon2 polygon2;
		polygon2.AddVertexCheckForDuplicates({ 0, 0 });
		polygon2.AddVertexCheckForDuplicates({ 1, 0 });
		polygon2.AddVertexCheckForDuplicates({ 1, 1 });
		polygon2.AddVertexCheckForDuplicates({ 0, 1 });
		polygon2.AddVertexCheckForDuplicates({ -1, 1 });

		EXPECT_EQ(polygon2.GetVertexNum(), 5);
		EXPECT_EQ(polygon2.GetVertex(0), designlab::Vector2(0, 0));
		EXPECT_EQ(polygon2.GetVertex(1), designlab::Vector2(1, 0));
		EXPECT_EQ(polygon2.GetVertex(2), designlab::Vector2(1, 1));
		EXPECT_EQ(polygon2.GetVertex(3), designlab::Vector2(0, 1));
		EXPECT_EQ(polygon2.GetVertex(4), designlab::Vector2(-1, 1));
	}

	//removeVertex�֐��̃e�X�g
	TEST(Polygon2, RemoveVertex)
	{
		//�l�p�`���쐬���C���_���폜����
		designlab::Polygon2 polygon;
		polygon.AddVertex({ 0, 0 });
		polygon.AddVertex({ 1, 0 });
		polygon.AddVertex({ 1, 1 });
		polygon.AddVertex({ 0, 1 });

		polygon.RemoveVertex(0);
		EXPECT_EQ(polygon.GetVertexNum(), 3);
		EXPECT_EQ(polygon.GetVertex(0), designlab::Vector2(1, 0));
		EXPECT_EQ(polygon.GetVertex(1), designlab::Vector2(1, 1));
		EXPECT_EQ(polygon.GetVertex(2), designlab::Vector2(0, 1));

		polygon.RemoveVertex(1);
		EXPECT_EQ(polygon.GetVertexNum(), 2);
		EXPECT_EQ(polygon.GetVertex(0), designlab::Vector2(1, 0));
		EXPECT_EQ(polygon.GetVertex(1), designlab::Vector2(0, 1));

		polygon.RemoveVertex(1);
		EXPECT_EQ(polygon.GetVertexNum(), 1);
		EXPECT_EQ(polygon.GetVertex(0), designlab::Vector2(1, 0));

		polygon.RemoveVertex(0);
		EXPECT_EQ(polygon.GetVertexNum(), 0);

		//���݂��Ȃ����_���w�肵���ꍇ�������Ȃ��D�l�p�`���쐬���C���݂��Ȃ����_���w�肷��
		designlab::Polygon2 polygon2;
		polygon2.AddVertex({ 0, 0 });
		polygon2.AddVertex({ 1, 0 });
		polygon2.AddVertex({ 1, 1 });
		polygon2.AddVertex({ 0, 1 });

		polygon2.RemoveVertex(4);
		EXPECT_EQ(polygon2.GetVertexNum(), 4);
		EXPECT_EQ(polygon2.GetVertex(0), designlab::Vector2(0, 0));
		EXPECT_EQ(polygon2.GetVertex(1), designlab::Vector2(1, 0));
		EXPECT_EQ(polygon2.GetVertex(2), designlab::Vector2(1, 1));
		EXPECT_EQ(polygon2.GetVertex(3), designlab::Vector2(0, 1));

		polygon2.RemoveVertex(5);
		EXPECT_EQ(polygon2.GetVertexNum(), 4);
		EXPECT_EQ(polygon2.GetVertex(0), designlab::Vector2(0, 0));
		EXPECT_EQ(polygon2.GetVertex(1), designlab::Vector2(1, 0));
		EXPECT_EQ(polygon2.GetVertex(2), designlab::Vector2(1, 1));
		EXPECT_EQ(polygon2.GetVertex(3), designlab::Vector2(0, 1));

		//���̒l���w�肵���ꍇ
		polygon2.RemoveVertex(-1);
		EXPECT_EQ(polygon2.GetVertexNum(), 4);
		EXPECT_EQ(polygon2.GetVertex(0), designlab::Vector2(0, 0));
		EXPECT_EQ(polygon2.GetVertex(1), designlab::Vector2(1, 0));
		EXPECT_EQ(polygon2.GetVertex(2), designlab::Vector2(1, 1));
		EXPECT_EQ(polygon2.GetVertex(3), designlab::Vector2(0, 1));
	}

	//removeLastVertex�֐��̃e�X�g
	TEST(Polygon2, RemoveLastVertex)
	{
		//�l�p�`���쐬���C���_���폜����
		designlab::Polygon2 polygon;
		polygon.AddVertex({ 0, 0 });
		polygon.AddVertex({ 1, 0 });
		polygon.AddVertex({ 1, 1 });
		polygon.AddVertex({ 0, 1 });

		polygon.RemoveLastVertex();
		EXPECT_EQ(polygon.GetVertexNum(), 3);
		EXPECT_EQ(polygon.GetVertex(0), designlab::Vector2(0, 0));
		EXPECT_EQ(polygon.GetVertex(1), designlab::Vector2(1, 0));
		EXPECT_EQ(polygon.GetVertex(2), designlab::Vector2(1, 1));

		polygon.RemoveLastVertex();
		EXPECT_EQ(polygon.GetVertexNum(), 2);
		EXPECT_EQ(polygon.GetVertex(0), designlab::Vector2(0, 0));
		EXPECT_EQ(polygon.GetVertex(1), designlab::Vector2(1, 0));

		polygon.RemoveLastVertex();
		EXPECT_EQ(polygon.GetVertexNum(), 1);
		EXPECT_EQ(polygon.GetVertex(0), designlab::Vector2(0, 0));

		polygon.RemoveLastVertex();
		EXPECT_EQ(polygon.GetVertexNum(), 0);

		//6�p�`���쐬���C���_���폜����
		designlab::Polygon2 polygon2;
		polygon2.AddVertex({ 0, 0 });
		polygon2.AddVertex({ 1, 0 });
		polygon2.AddVertex({ 1, 1 });
		polygon2.AddVertex({ 0, 1 });
		polygon2.AddVertex({ -1, 1 });
		polygon2.AddVertex({ -1, 0 });

		polygon2.RemoveLastVertex();
		EXPECT_EQ(polygon2.GetVertexNum(), 5);
		EXPECT_EQ(polygon2.GetVertex(0), designlab::Vector2(0, 0));
		EXPECT_EQ(polygon2.GetVertex(1), designlab::Vector2(1, 0));
		EXPECT_EQ(polygon2.GetVertex(2), designlab::Vector2(1, 1));
		EXPECT_EQ(polygon2.GetVertex(3), designlab::Vector2(0, 1));
		EXPECT_EQ(polygon2.GetVertex(4), designlab::Vector2(-1, 1));
	}

	//getVertexNum�֐��̃e�X�g
	TEST(Polygon2, GetVertexNum)
	{
		//���_���Ȃ����p�`���쐬���C���_�����m�F����
		designlab::Polygon2 polygon2;
		EXPECT_EQ(polygon2.GetVertexNum(), 0);

		//4�p�`���쐬���C���_�����m�F����
		designlab::Polygon2 polygon;
		polygon.AddVertex({ 0, 0 });
		polygon.AddVertex({ 1, 0 });
		polygon.AddVertex({ 1, 1 });
		polygon.AddVertex({ 0, 1 });

		EXPECT_EQ(polygon.GetVertexNum(), 4);

		//���_��S�č폜���C���_�����m�F����
		polygon.Reset();

		EXPECT_EQ(polygon.GetVertexNum(), 0);

		//6�p�`���쐬���C���_�����m�F����
		designlab::Polygon2 polygon3;
		polygon3.AddVertex({ 0, 0 });
		polygon3.AddVertex({ 1, 0 });
		polygon3.AddVertex({ 1, 1 });
		polygon3.AddVertex({ 0, 1 });
		polygon3.AddVertex({ -1, 1 });
		polygon3.AddVertex({ -1, 0 });

		EXPECT_EQ(polygon3.GetVertexNum(), 6);

		//���_���폜���C���_�����m�F����
		polygon3.RemoveVertex(0);
		EXPECT_EQ(polygon3.GetVertexNum(), 5);
		polygon3.RemoveVertex(0);
		EXPECT_EQ(polygon3.GetVertexNum(), 4);
		polygon3.RemoveVertex(0);
		EXPECT_EQ(polygon3.GetVertexNum(), 3);
		polygon3.RemoveVertex(0);
		EXPECT_EQ(polygon3.GetVertexNum(), 2);
		polygon3.RemoveVertex(0);
		EXPECT_EQ(polygon3.GetVertexNum(), 1);
		polygon3.RemoveVertex(0);
		EXPECT_EQ(polygon3.GetVertexNum(), 0);
	}

	//getVertex�֐��̃e�X�g
	TEST(Polygon2, GetVertex)
	{
		//4�p�`���쐬���C���̒��_�̍��W���m�F����
		designlab::Polygon2 polygon;
		polygon.AddVertex({ 0, 0 });
		polygon.AddVertex({ 1, 0 });
		polygon.AddVertex({ 1, 1 });
		polygon.AddVertex({ 0, 1 });

		EXPECT_EQ(polygon.GetVertex(0), designlab::Vector2(0, 0));
		EXPECT_EQ(polygon.GetVertex(1), designlab::Vector2(1, 0));
		EXPECT_EQ(polygon.GetVertex(2), designlab::Vector2(1, 1));
		EXPECT_EQ(polygon.GetVertex(3), designlab::Vector2(0, 1));

		//6�p�`���쐬���C���̒��_�̍��W���m�F����
		designlab::Polygon2 polygon2;
		polygon2.AddVertex({ 0, 0 });
		polygon2.AddVertex({ 1, 0 });
		polygon2.AddVertex({ 1, 1 });
		polygon2.AddVertex({ 0, 1 });
		polygon2.AddVertex({ -1, 1 });
		polygon2.AddVertex({ -1, 0 });

		EXPECT_EQ(polygon2.GetVertex(0), designlab::Vector2(0, 0));
		EXPECT_EQ(polygon2.GetVertex(1), designlab::Vector2(1, 0));
		EXPECT_EQ(polygon2.GetVertex(2), designlab::Vector2(1, 1));
		EXPECT_EQ(polygon2.GetVertex(3), designlab::Vector2(0, 1));
		EXPECT_EQ(polygon2.GetVertex(4), designlab::Vector2(-1, 1));
		EXPECT_EQ(polygon2.GetVertex(5), designlab::Vector2(-1, 0));
	}

	//IsConvex�֐��̃e�X�g
	TEST(Polygon2, IsConvexRightTurn)
	{
		//4�p�`���쐬���C�ʑ��p�`���ǂ������m�F����
		designlab::Polygon2 polygon;
		polygon.AddVertex({ 0, 0 });
		polygon.AddVertex({ 1, 0 });
		polygon.AddVertex({ 1, 1 });
		polygon.AddVertex({ 0, 1 });

		EXPECT_TRUE(polygon.IsConvex());

		//6�p�`���쐬���C�ʑ��p�`���ǂ������m�F����
		designlab::Polygon2 polygon2;
		polygon2.AddVertex({ 0, -0.5f });
		polygon2.AddVertex({ 1, 0 });
		polygon2.AddVertex({ 1, 1 });
		polygon2.AddVertex({ 0, 1.5f });
		polygon2.AddVertex({ -1, 1 });
		polygon2.AddVertex({ -1, 0 });

		EXPECT_TRUE(polygon2.IsConvex());

		//�ʑ��p�`�łȂ����p�`���쐬���C�ʑ��p�`���ǂ������m�F����
		designlab::Polygon2 polygon3;
		polygon3.AddVertex({ 0, 0 });
		polygon3.AddVertex({ 1, 0 });
		polygon3.AddVertex({ 1, 1 });
		polygon3.AddVertex({ 0, 1 });
		polygon3.AddVertex({ 0.5f, 0.5f });

		EXPECT_FALSE(polygon3.IsConvex());

		//���_����3�����̑��p�`���쐬���C�ʑ��p�`���ǂ������m�F����
		designlab::Polygon2 polygon4;
		polygon4.AddVertex({ 0, 0 });
		polygon4.AddVertex({ 1, 0 });

		EXPECT_FALSE(polygon4.IsConvex());
	}

	//IsConvex�֐��̃e�X�g(�����)
	TEST(Polygon2, IsConvexLeftTurn)
	{
		//4�p�`���쐬���C�ʑ��p�`���ǂ������m�F����
		designlab::Polygon2 polygon;
		polygon.AddVertex({ 0, 0 });
		polygon.AddVertex({ 0, 1 });
		polygon.AddVertex({ 1, 1 });
		polygon.AddVertex({ 1, 0 });

		EXPECT_TRUE(polygon.IsConvex());

		//6�p�`���쐬���C�ʑ��p�`���ǂ������m�F����
		designlab::Polygon2 polygon2;
		polygon2.AddVertex({ 0, -0.5f });
		polygon2.AddVertex({ -1, 0 });
		polygon2.AddVertex({ -1, 1 });
		polygon2.AddVertex({ 0, 1.5f });
		polygon2.AddVertex({ 1, 1 });
		polygon2.AddVertex({ 1, 0 });

		EXPECT_TRUE(polygon2.IsConvex());

		//�ʑ��p�`�łȂ����p�`���쐬���C�ʑ��p�`���ǂ������m�F����
		designlab::Polygon2 polygon3;
		polygon3.AddVertex({ 0, 0 });
		polygon3.AddVertex({ 0, 1 });
		polygon3.AddVertex({ 1, 1 });
		polygon3.AddVertex({ 1, 0 });
		polygon3.AddVertex({ 0.5f, 0.5f });


		EXPECT_FALSE(polygon3.IsConvex());

		//���_����3�����̑��p�`���쐬���C�ʑ��p�`���ǂ������m�F����
		designlab::Polygon2 polygon4;
		polygon4.AddVertex({ 0, 0 });
		polygon4.AddVertex({ 1, 0 });

		EXPECT_FALSE(polygon4.IsConvex());
	}

}	//namespace dl_vec_test
