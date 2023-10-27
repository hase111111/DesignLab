#include "pch.h"

#include "../DesignLab/designlab_vector3.h"
#include "../DesignLab/designlab_vector3.cpp"	//�ǂ���1�̃t�@�C���ŁCcpp�t�@�C�����C���N���[�h����K�v������


namespace dl = ::designlab;


namespace designlab::test::common::math
{
	TEST(Vector3Test, DefaultConstructor)
	{
		//�f�t�H���g�R���X�g���N�^�̃e�X�g
		dl::Vector3 vec;

		std::string str = "�f�t�H���g�R���X�g���N�^��(0,0,0)�ƂȂ�";
		EXPECT_FLOAT_EQ(vec.x, 0.f) << str;
		EXPECT_FLOAT_EQ(vec.y, 0.f) << str;
		EXPECT_FLOAT_EQ(vec.z, 0.f) << str;
	}

	TEST(Vector3Test, Constructor)
	{
		//�R���X�g���N�^�̃e�X�g
		dl::Vector3 vec(1.f, 2.f, 3.f);

		std::string str = "�R���X�g���N�^�͗^����ꂽ�l�ŁCVector3���쐬����";
		EXPECT_FLOAT_EQ(vec.x, 1.f) << str;
		EXPECT_FLOAT_EQ(vec.y, 2.f) << str;
		EXPECT_FLOAT_EQ(vec.z, 3.f) << str;
	}

	TEST(Vector3Test, CopyConstructor)
	{
		//�R�s�[�R���X�g���N�^�̃e�X�g
		dl::Vector3 vec(1.f, 2.f, 3.f);
		dl::Vector3 vec2(vec);

		std::string str = "�R�s�[�R���X�g���N�^�͗^����ꂽ�l���R�s�[���āCVector3���쐬����";
		EXPECT_FLOAT_EQ(vec2.x, 1.f) << str;
		EXPECT_FLOAT_EQ(vec2.y, 2.f) << str;
		EXPECT_FLOAT_EQ(vec2.z, 3.f) << str;
	}

	TEST(Vector3Test, AssignmentOperator)
	{
		// Assignment Operator = ������Z�q
		dl::Vector3 vec(1.f, 2.f, 3.f);
		dl::Vector3 vec2 = vec;

		std::string str = "������Z�q�͗^����ꂽ�l�������āCVector3���쐬����";
		EXPECT_FLOAT_EQ(vec2.x, 1.f) << str;
		EXPECT_FLOAT_EQ(vec2.y, 2.f) << str;
		EXPECT_FLOAT_EQ(vec2.z, 3.f) << str;
	}

}	// namespace designlab::test::common::math
