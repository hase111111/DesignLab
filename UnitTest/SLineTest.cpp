#include "pch.h"
#include "../DesignLab/MyVector2.h"

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

		//�ӂ̒��������肸�C�ڂ��Ă��Ȃ��ꍇ��_�͂Ȃ��C(0,0)���Ԃ�
		line1 = SLine2({ 0, 0 }, { 1, 1 });
		line2 = SLine2({ -5,-6 }, { -3,-1 });
		EXPECT_EQ(line1.getIntersection(line2), SVector2(0, 0));

		//�[�_�Őڂ���ꍇ�C���̓_���Ԃ�D
		line1 = SLine2({ 0, 0 }, { 1, 1 });
		line2 = SLine2({ 1, 1 }, { 2, 3 });
		EXPECT_EQ(line1.getIntersection(line2), SVector2(1, 1));
	}
}