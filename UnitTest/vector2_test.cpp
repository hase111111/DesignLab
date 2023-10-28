#include "pch.h"

#include "../DesignLab/designlab_vector2.h"
#include "../DesignLab/designlab_vector2.cpp"	//�ǂ���1�̃t�@�C���ŁCcpp�t�@�C�����C���N���[�h����K�v������


namespace dl = ::designlab;


namespace designlab::test::common::math
{
	TEST(Vector2Test, DefaultConstructorTest)
	{
		//�f�t�H���g�R���X�g���N�^�̃e�X�g
		dl::Vector2 vec;

		std::string str = "�f�t�H���g�R���X�g���N�^��(0,0)�ƂȂ�";
		EXPECT_FLOAT_EQ(vec.x, 0.f) << str;
		EXPECT_FLOAT_EQ(vec.y, 0.f) << str;
	}

	TEST(Vector2Test, ConstructorTest)
	{
		//�R���X�g���N�^�̃e�X�g
		dl::Vector2 vec(1.f, 2.f);

		std::string str = "�R���X�g���N�^�͗^����ꂽ�l�ŁCVector2���쐬����";
		EXPECT_FLOAT_EQ(vec.x, 1.f) << str;
		EXPECT_FLOAT_EQ(vec.y, 2.f) << str;
	}

	TEST(Vector2Test, CopyConstructorTest)
	{
		//�R�s�[�R���X�g���N�^�̃e�X�g
		dl::Vector2 vec(1.f, 2.f);
		dl::Vector2 vec2(vec);

		std::string str = "�R�s�[�R���X�g���N�^�͗^����ꂽ�l���R�s�[���āCVector2���쐬����";
		EXPECT_FLOAT_EQ(vec2.x, 1.f) << str;
		EXPECT_FLOAT_EQ(vec2.y, 2.f) << str;
	}

	TEST(Vector2Test, MoveConstructorTest)
	{
		//���[�u�R���X�g���N�^�̃e�X�g
		dl::Vector2 vec(1.f, 2.f);
		dl::Vector2 vec2(std::move(vec));

		std::string str = "���[�u�R���X�g���N�^�͗^����ꂽ�l�����[�u���āCVector2���쐬����";
		EXPECT_FLOAT_EQ(vec2.x, 1.f) << str;
		EXPECT_FLOAT_EQ(vec2.y, 2.f) << str;
	}

	TEST(Vector2Test, AssignmentOperatorTest)
	{
		// Assignment Operator = ������Z�q
		dl::Vector2 vec(1.f, 2.f);
		dl::Vector2 vec2 = vec;

		std::string str = "������Z�q�͗^����ꂽ�l�������āCVector2���쐬����";
		EXPECT_FLOAT_EQ(vec2.x, 1.f) << str;
		EXPECT_FLOAT_EQ(vec2.y, 2.f) << str;
	}

}	// namespace designlab::test::common::math