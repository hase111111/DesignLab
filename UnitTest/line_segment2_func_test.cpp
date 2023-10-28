#include "pch.h"

#include "../DesignLab/designlab_line_segment2.h"


namespace dl = ::designlab;


namespace 
{
	//! @brief �������쐬����
	dl::LineSegment2 MakeLineSegment2(const dl::Vector2& start, const dl::Vector2& end)
	{
		dl::LineSegment2 res;
		res.start = start;
		res.end = end;
		return res;
	}
}


namespace designlab::test::common::math
{
	TEST(LineSegment2Test, GetLengthTest)
	{
		const std::vector<std::tuple<dl::LineSegment2, float>> testcase_list{
			{ MakeLineSegment2({ 0.f, 0.f }, { 1.f, 1.f }),		sqrt(2.0f) },
			{ MakeLineSegment2({ 0.f, -3.5f }, { 1.f, -3.5f}),	1.0f },
			{ MakeLineSegment2({ -4.f, 10.f }, { -4.f, 11.f}),	1.0f },
			{ MakeLineSegment2({ 0.f, 0.f }, { 0.f, 0.f}),			0.0f },
			{ MakeLineSegment2({ 100.f, -30.f }, { 100.f, -30.f}),	0.0f },
			{ MakeLineSegment2({ 0.f, 0.f }, { 1.f, 2.f}),	sqrt(5.0f) },
			{ MakeLineSegment2({ 1.f, 2.f }, { 0.f, 0.f}),	sqrt(5.0f) },
			{ MakeLineSegment2({ 2.f, 1.f }, { 0.f, 0.f}),	sqrt(5.0f) },
			{ MakeLineSegment2({ 0.f, 0.f }, { 2.f, 2.f}),	sqrt(8.0f) },
		};

		for (const auto& i : testcase_list)
		{
			const auto& line = std::get<0>(i);
			const auto& expected = std::get<1>(i);

			std::string error_message = " start = " + line.start.ToString() + "\n end   = " + line.end.ToString() + "\n";

			EXPECT_FLOAT_EQ(line.GetLength(), expected) << error_message;
		}
	}

	TEST(LineSegment2Test, IsParallelTest)
	{
		const std::vector<std::tuple<dl::LineSegment2, dl::LineSegment2, bool>> testcase_list{
			{ MakeLineSegment2({ 0, 0 }, {1,1}), MakeLineSegment2({ 0, 1 }, { 1, 2 }), true},
			{ MakeLineSegment2({ 10, 0 }, { 0, 0 }), MakeLineSegment2({ 0, 0 }, { -400, 0 }), true},
			{ MakeLineSegment2({ 35, 70 }, { 35, -200 }), MakeLineSegment2({ -14.7f, -300 }, { -14.7f, 2000 }), true},
			{ MakeLineSegment2({ 0, 0 }, { 1, 1 }), MakeLineSegment2({ 0, 1 }, { 1, 0 }), false},
			{ MakeLineSegment2({ 0, 0 }, { 1, 1 }), MakeLineSegment2({ 1, 0 }, { 0, 1 }), false},
			{ MakeLineSegment2({ 1,1 }, { 0,0 }), MakeLineSegment2({ 0,1 }, { 1,0 }), false},
			{ MakeLineSegment2({ 0, 0 }, { 1, 1 }), MakeLineSegment2({ -5,-6 }, { -3,-1 }), false},
			{ MakeLineSegment2({ 0, 0 }, { 1, 1 }), MakeLineSegment2({ 1, 1 }, { 2, 3 }), false},
		};

		for (const auto& i : testcase_list)
		{
			const auto& line1 = std::get<0>(i);
			const auto& line2 = std::get<1>(i);
			const auto& expected = std::get<2>(i);

			std::string error_message = " line1 = (" + line1.start.ToString() + ", " + line1.end.ToString() + ")\n" +
				" line2 = (" + line2.start.ToString() + ", " + line2.end.ToString() + ")\n";

			if (expected) 
			{
				EXPECT_TRUE(line1.IsParallel(line2)) << error_message;
			}
			else 
			{
				EXPECT_FALSE(line1.IsParallel(line2)) << error_message;
			}
		}
	}

	TEST(LineSegment2Test, GetIntersectionTest)
	{
		//�ӂ����s�̏ꍇ��_�͂Ȃ������ɂ���C(0,0)���Ԃ�
		designlab::LineSegment2 line1 = designlab::LineSegment2({ 0, 0 }, { 1, 1 });
		designlab::LineSegment2 line2 = designlab::LineSegment2({ 0, 1 }, { 1, 2 });
		EXPECT_EQ(line1.GetIntersection(line2), designlab::Vector2(0, 0));

		//�ӂ����s�łȂ��ꍇ��_������
		line1 = designlab::LineSegment2({ 0, 0 }, { 1, 1 });
		line2 = designlab::LineSegment2({ 0, 1 }, { 1, 0 });
		EXPECT_EQ(line1.GetIntersection(line2), designlab::Vector2(0.5, 0.5));

		//�ӂ̌������t�ł���_�͓���
		line1 = designlab::LineSegment2({ 0, 0 }, { 1, 1 });
		line2 = designlab::LineSegment2({ 1, 0 }, { 0, 1 });
		EXPECT_EQ(line1.GetIntersection(line2), designlab::Vector2(0.5, 0.5));

		line1 = designlab::LineSegment2({ 1,1 }, { 0,0 });
		line2 = designlab::LineSegment2({ 0,1 }, { 1,0 });
		EXPECT_EQ(line1.GetIntersection(line2), designlab::Vector2(0.5, 0.5));

		//�ӂ̒��������肸�C�ڂ��Ă��Ȃ��ꍇ��_�͂Ȃ��C(0,0)���Ԃ�
		line1 = designlab::LineSegment2({ 0, 0 }, { 1, 1 });
		line2 = designlab::LineSegment2({ -5,-6 }, { -3,-1 });
		EXPECT_EQ(line1.GetIntersection(line2), designlab::Vector2(0, 0));

		//�[�_�Őڂ���ꍇ�C���̓_���Ԃ�D
		line1 = designlab::LineSegment2({ 0, 0 }, { 1, 1 });
		line2 = designlab::LineSegment2({ 1, 1 }, { 2, 3 });
		EXPECT_EQ(line1.GetIntersection(line2), designlab::Vector2(1, 1));
	}

	TEST(LineSegment2Test, HasIntersectionTest)
	{
		//�ӂ����s�̏ꍇ��_�͂Ȃ�
		designlab::LineSegment2 line1 = designlab::LineSegment2({ 0, 0 }, { 1, 1 });
		designlab::LineSegment2 line2 = designlab::LineSegment2({ 0, 1 }, { 1, 2 });
		EXPECT_FALSE(line1.HasIntersection(line2));

		//�ӂ����s�łȂ��ꍇ��_������
		line1 = designlab::LineSegment2({ 0, 0 }, { 1, 1 });
		line2 = designlab::LineSegment2({ 0, 1 }, { 1, 0 });
		EXPECT_TRUE(line1.HasIntersection(line2));

		//�ӂ̌������t�ł���_�͓���
		line1 = designlab::LineSegment2({ 0, 0 }, { 1, 1 });
		line2 = designlab::LineSegment2({ 1, 0 }, { 0, 1 });
		EXPECT_TRUE(line1.HasIntersection(line2));

		line1 = designlab::LineSegment2({ 1,1 }, { 0,0 });
		line2 = designlab::LineSegment2({ 0,1 }, { 1,0 });
		EXPECT_TRUE(line1.HasIntersection(line2));

		//�ӂ̒��������肸�C�ڂ��Ă��Ȃ��ꍇ��_�͂Ȃ�
		line1 = designlab::LineSegment2({ 0, 0 }, { 1, 1 });
		line2 = designlab::LineSegment2({ -5,-6 }, { -3,-1 });
		EXPECT_FALSE(line1.HasIntersection(line2));

		//�[�_�Őڂ���ꍇ�C���̓_���Ԃ�D
		line1 = designlab::LineSegment2({ 0, 0 }, { 1, 1 });
		line2 = designlab::LineSegment2({ 1, 1 }, { 2, 3 });
		EXPECT_TRUE(line1.HasIntersection(line2));
	}

}	//namespace dl_vec_test