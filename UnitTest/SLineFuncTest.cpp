#include "pch.h"
#include "../DesignLab/MyLine.h"
#include "../DesignLab/MyLine.cpp"

using namespace my_vec;

namespace SLine2Test
{
	TEST(SLine2FuncTest, GetIntersection)
	{
		//�ӂ����s�̏ꍇ��_�͂Ȃ��C(0,0)���Ԃ�
		SLine2 line1 = SLine2({ 0, 0 }, { 1, 1 });
		SLine2 line2 = SLine2({ 0, 1 }, { 1, 2 });
		EXPECT_EQ(line1.getIntersection(line2), SVector2(0, 0));

		//�ӂ����s�łȂ��ꍇ��_������
		line1 = SLine2({ 0, 0 }, { 1, 1 });
		line2 = SLine2({ 0, 1 }, { 1, 0 });
		EXPECT_EQ(line1.getIntersection(line2), SVector2(0.5, 0.5));

		//�ӂ̌������t�ł���_�͓���
		line1 = SLine2({ 0, 0 }, { 1, 1 });
		line2 = SLine2({ 1, 0 }, { 0, 1 });
		EXPECT_EQ(line1.getIntersection(line2), SVector2(0.5, 0.5));

		line1 = SLine2({ 1,1 }, { 0,0 });
		line2 = SLine2({ 0,1 }, { 1,0 });
		EXPECT_EQ(line1.getIntersection(line2), SVector2(0.5, 0.5));

		//�ӂ̒��������肸�C�ڂ��Ă��Ȃ��ꍇ��_�͂Ȃ��C(0,0)���Ԃ�
		line1 = SLine2({ 0, 0 }, { 1, 1 });
		line2 = SLine2({ -5,-6 }, { -3,-1 });
		EXPECT_EQ(line1.getIntersection(line2), SVector2(0, 0));

		//�[�_�Őڂ���ꍇ�C���̓_���Ԃ�D
		line1 = SLine2({ 0, 0 }, { 1, 1 });
		line2 = SLine2({ 1, 1 }, { 2, 3 });
		EXPECT_EQ(line1.getIntersection(line2), SVector2(1, 1));
	}

	//hasIntersection�֐��̃e�X�g
	TEST(SLine2FuncTest, HasIntersection)
	{
		//�ӂ����s�̏ꍇ��_�͂Ȃ�
		SLine2 line1 = SLine2({ 0, 0 }, { 1, 1 });
		SLine2 line2 = SLine2({ 0, 1 }, { 1, 2 });
		EXPECT_FALSE(line1.hasIntersection(line2));

		//�ӂ����s�łȂ��ꍇ��_������
		line1 = SLine2({ 0, 0 }, { 1, 1 });
		line2 = SLine2({ 0, 1 }, { 1, 0 });
		EXPECT_TRUE(line1.hasIntersection(line2));

		//�ӂ̌������t�ł���_�͓���
		line1 = SLine2({ 0, 0 }, { 1, 1 });
		line2 = SLine2({ 1, 0 }, { 0, 1 });
		EXPECT_TRUE(line1.hasIntersection(line2));

		line1 = SLine2({ 1,1 }, { 0,0 });
		line2 = SLine2({ 0,1 }, { 1,0 });
		EXPECT_TRUE(line1.hasIntersection(line2));

		//�ӂ̒��������肸�C�ڂ��Ă��Ȃ��ꍇ��_�͂Ȃ�
		line1 = SLine2({ 0, 0 }, { 1, 1 });
		line2 = SLine2({ -5,-6 }, { -3,-1 });
		EXPECT_FALSE(line1.hasIntersection(line2));

		//�[�_�Őڂ���ꍇ�C���̓_���Ԃ�D
		line1 = SLine2({ 0, 0 }, { 1, 1 });
		line2 = SLine2({ 1, 1 }, { 2, 3 });
		EXPECT_TRUE(line1.hasIntersection(line2));
	}

	//getLength�֐��̃e�X�g
	TEST(SLine2FuncTest, GetLength)
	{
		//�ӂ̒������������Ԃ�
		SLine2 line1 = SLine2({ 0, 0 }, { 1, 1 });
		EXPECT_FLOAT_EQ(line1.getLength(), sqrt(2));

		line1 = SLine2({ 0, 0 }, { 1, 0 });
		EXPECT_FLOAT_EQ(line1.getLength(), 1);

		line1 = SLine2({ 0, 0 }, { 0, 1 });
		EXPECT_FLOAT_EQ(line1.getLength(), 1);

		line1 = SLine2({ 0, 0 }, { 0, 0 });
		EXPECT_FLOAT_EQ(line1.getLength(), 0);

		line1 = SLine2({ 0, 0 }, { 1, 2 });
		EXPECT_FLOAT_EQ(line1.getLength(), sqrt(5));

		line1 = SLine2({ 0, 0 }, { 2, 1 });
		EXPECT_FLOAT_EQ(line1.getLength(), sqrt(5));

		line1 = SLine2({ 0, 0 }, { 2, 2 });
		EXPECT_FLOAT_EQ(line1.getLength(), sqrt(8));
	}

	//isParallel�֐��̃e�X�g
	TEST(SLine2FuncTest, IsParallel)
	{
		//�ӂ����s�̏ꍇtrue���Ԃ�
		SLine2 line1 = SLine2({ 0, 0 }, { 1, 1 });
		SLine2 line2 = SLine2({ 0, 1 }, { 1, 2 });
		EXPECT_TRUE(line1.isParallel(line2));

		line1 = SLine2({ 10, 0 }, { 0, 0 });
		line2 = SLine2({ 0, 0 }, { -400, 0 });
		EXPECT_TRUE(line1.isParallel(line2));

		line1 = SLine2({ 35, 70 }, { 35, -200 });
		line2 = SLine2({ -14.7f, -300 }, { -14.7f, 2000 });
		EXPECT_TRUE(line1.isParallel(line2));


		//�ӂ����s�łȂ��ꍇfalse���Ԃ�
		line1 = SLine2({ 0, 0 }, { 1, 1 });
		line2 = SLine2({ 0, 1 }, { 1, 0 });
		EXPECT_FALSE(line1.isParallel(line2));

		line1 = SLine2({ 0, 0 }, { 1, 1 });
		line2 = SLine2({ 1, 0 }, { 0, 1 });
		EXPECT_FALSE(line1.isParallel(line2));

		line1 = SLine2({ 1,1 }, { 0,0 });
		line2 = SLine2({ 0,1 }, { 1,0 });
		EXPECT_FALSE(line1.isParallel(line2));

		line1 = SLine2({ 0, 0 }, { 1, 1 });
		line2 = SLine2({ -5,-6 }, { -3,-1 });
		EXPECT_FALSE(line1.isParallel(line2));

		line1 = SLine2({ 0, 0 }, { 1, 1 });
		line2 = SLine2({ 1, 1 }, { 2, 3 });
		EXPECT_FALSE(line1.isParallel(line2));
	}
}