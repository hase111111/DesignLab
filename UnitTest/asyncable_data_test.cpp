#include "pch.h"

#include "../DesignLab/asyncable_data.h"


// �񓯊������̃e�X�g�͍s�킸�C�P��̃X���b�h�ł̃e�X�g�̂ݍs���D


namespace designlab::test::common 
{
	TEST(AsyncableDataTestIntType, DefaultConstructorTest)
	{
		AsyncableData<int> data;	//������

		EXPECT_EQ(data.GetUpdateCount(), 0) << "�R���X�g���N�^�ŏ����������ꍇ�C�X�V�񐔂�0�ł��D";
	}

	TEST(AsyncableDataTestIntType, ConstructorTest)
	{
		AsyncableData<int> data(-4);	//������

		EXPECT_EQ(data.GetData(), -4) << "�R���X�g���N�^�Œl���������ł��܂��D";
		EXPECT_EQ(data.GetUpdateCount(), 0) << "�R���X�g���N�^�ŏ����������ꍇ�C�X�V�񐔂�0�ł��D";
	}

	TEST(AsyncableDataTestIntType, GetUpdateCountTest)
	{
		AsyncableData<int> data(0);	//������

		EXPECT_EQ(data.GetUpdateCount(), 0) << "���������_�ł́C�X�V�񐔂�0�ł��D";

		data.SetData(10);
		EXPECT_EQ(data.GetUpdateCount(), 1) << "�f�[�^���X�V����ƁC�X�V�񐔂�1�������܂��D";

		data.SetData(20);
		EXPECT_EQ(data.GetUpdateCount(), 2) << "�f�[�^���X�V����ƁC�X�V�񐔂�1�������܂��D";
	}

	TEST(AsyncableDataTestIntType, TestCanGetAndSetData)
	{
		//������
		AsyncableData<int> data(0);

		data.SetData(10);
		EXPECT_EQ(data.GetData(), 10);

		data.SetData(20);
		EXPECT_EQ(data.GetData(), 20);
	}
}