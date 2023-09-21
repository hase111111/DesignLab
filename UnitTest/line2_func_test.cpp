#include "pch.h"

#include "../DesignLab/designlab_line_segment2.h"
#include "../DesignLab/designlab_line.cpp"



namespace dl_vec_test
{
	TEST(LineSegment2, GetIntersection)
	{
		//�ӂ����s�̏ꍇ��_�͂Ȃ��C(0,0)���Ԃ�
		designlab::LineSegment2 line1 = designlab::LineSegment2({ 0, 0 }, { 1, 1 });
		designlab::LineSegment2 line2 = designlab::LineSegment2({ 0, 1 }, { 1, 2 });
		EXPECT_EQ(line1.GetIntersection(line2), designlab::SVector2(0, 0));

		//�ӂ����s�łȂ��ꍇ��_������
		line1 = designlab::LineSegment2({ 0, 0 }, { 1, 1 });
		line2 = designlab::LineSegment2({ 0, 1 }, { 1, 0 });
		EXPECT_EQ(line1.GetIntersection(line2), designlab::SVector2(0.5, 0.5));

		//�ӂ̌������t�ł���_�͓���
		line1 = designlab::LineSegment2({ 0, 0 }, { 1, 1 });
		line2 = designlab::LineSegment2({ 1, 0 }, { 0, 1 });
		EXPECT_EQ(line1.GetIntersection(line2), designlab::SVector2(0.5, 0.5));

		line1 = designlab::LineSegment2({ 1,1 }, { 0,0 });
		line2 = designlab::LineSegment2({ 0,1 }, { 1,0 });
		EXPECT_EQ(line1.GetIntersection(line2), designlab::SVector2(0.5, 0.5));

		//�ӂ̒��������肸�C�ڂ��Ă��Ȃ��ꍇ��_�͂Ȃ��C(0,0)���Ԃ�
		line1 = designlab::LineSegment2({ 0, 0 }, { 1, 1 });
		line2 = designlab::LineSegment2({ -5,-6 }, { -3,-1 });
		EXPECT_EQ(line1.GetIntersection(line2), designlab::SVector2(0, 0));

		//�[�_�Őڂ���ꍇ�C���̓_���Ԃ�D
		line1 = designlab::LineSegment2({ 0, 0 }, { 1, 1 });
		line2 = designlab::LineSegment2({ 1, 1 }, { 2, 3 });
		EXPECT_EQ(line1.GetIntersection(line2), designlab::SVector2(1, 1));
	}

	//hasIntersection�֐��̃e�X�g
	TEST(LineSegment2, HasIntersection)
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

	//getLength�֐��̃e�X�g
	TEST(LineSegment2, GetLength)
	{
		//�ӂ̒������������Ԃ�
		designlab::LineSegment2 line1 = designlab::LineSegment2({ 0, 0 }, { 1, 1 });
		EXPECT_FLOAT_EQ(line1.GetLength(), sqrt(2.0f));

		line1 = designlab::LineSegment2({ 0, 0 }, { 1, 0 });
		EXPECT_FLOAT_EQ(line1.GetLength(), 1.0f);

		line1 = designlab::LineSegment2({ 0, 0 }, { 0, 1 });
		EXPECT_FLOAT_EQ(line1.GetLength(), 1.0f);

		line1 = designlab::LineSegment2({ 0, 0 }, { 0, 0 });
		EXPECT_FLOAT_EQ(line1.GetLength(), 0.0f);

		line1 = designlab::LineSegment2({ 0, 0 }, { 1, 2 });
		EXPECT_FLOAT_EQ(line1.GetLength(), sqrt(5.0f));

		line1 = designlab::LineSegment2({ 0, 0 }, { 2, 1 });
		EXPECT_FLOAT_EQ(line1.GetLength(), sqrt(5.0f));

		line1 = designlab::LineSegment2({ 0, 0 }, { 2, 2 });
		EXPECT_FLOAT_EQ(line1.GetLength(), sqrt(8.0f));
	}

	//isParallel�֐��̃e�X�g
	TEST(LineSegment2, IsParallel)
	{
		//�ӂ����s�̏ꍇtrue���Ԃ�
		designlab::LineSegment2 line1 = designlab::LineSegment2({ 0, 0 }, { 1, 1 });
		designlab::LineSegment2 line2 = designlab::LineSegment2({ 0, 1 }, { 1, 2 });
		EXPECT_TRUE(line1.IsParallel(line2));

		line1 = designlab::LineSegment2({ 10, 0 }, { 0, 0 });
		line2 = designlab::LineSegment2({ 0, 0 }, { -400, 0 });
		EXPECT_TRUE(line1.IsParallel(line2));

		line1 = designlab::LineSegment2({ 35, 70 }, { 35, -200 });
		line2 = designlab::LineSegment2({ -14.7f, -300 }, { -14.7f, 2000 });
		EXPECT_TRUE(line1.IsParallel(line2));


		//�ӂ����s�łȂ��ꍇfalse���Ԃ�
		line1 = designlab::LineSegment2({ 0, 0 }, { 1, 1 });
		line2 = designlab::LineSegment2({ 0, 1 }, { 1, 0 });
		EXPECT_FALSE(line1.IsParallel(line2));

		line1 = designlab::LineSegment2({ 0, 0 }, { 1, 1 });
		line2 = designlab::LineSegment2({ 1, 0 }, { 0, 1 });
		EXPECT_FALSE(line1.IsParallel(line2));

		line1 = designlab::LineSegment2({ 1,1 }, { 0,0 });
		line2 = designlab::LineSegment2({ 0,1 }, { 1,0 });
		EXPECT_FALSE(line1.IsParallel(line2));

		line1 = designlab::LineSegment2({ 0, 0 }, { 1, 1 });
		line2 = designlab::LineSegment2({ -5,-6 }, { -3,-1 });
		EXPECT_FALSE(line1.IsParallel(line2));

		line1 = designlab::LineSegment2({ 0, 0 }, { 1, 1 });
		line2 = designlab::LineSegment2({ 1, 1 }, { 2, 3 });
		EXPECT_FALSE(line1.IsParallel(line2));
	}

}	//namespace dl_vec_test

