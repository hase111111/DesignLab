#include "pch.h"

#include "../DesignLab/asyncable_data.h"


// �񓯊������̃e�X�g�͍s�킸�C�P��̃X���b�h�ł̃e�X�g�̂ݍs���D


namespace designlab::test::common
{
	TEST(AsyncableDataTestVectorIntType, DefaultConstructorTest)
	{
		AsyncableData<std::vector<int>> data;	//������

		EXPECT_EQ(data.GetUpdateCount(), 0) << "�R���X�g���N�^�ŏ����������ꍇ�C�X�V�񐔂�0�ł��D";
		EXPECT_EQ(data.GetSize(), 0) << "�R���X�g���N�^�ŏ����������ꍇ�C�T�C�Y��0�ł��D";
	}

	TEST(AsyncableDataTestVectorIntType, ConstructorTest)
	{
		const std::vector<int> sample = { 1, 2, 3 };

		AsyncableData<std::vector<int>> data(sample);	//������

		EXPECT_EQ(data.GetData(), sample) << "�R���X�g���N�^�Œl���������ł��܂��D";
		EXPECT_EQ(data.GetUpdateCount(), 0) << "�R���X�g���N�^�ŏ����������ꍇ�C�X�V�񐔂�0�ł��D";
		EXPECT_EQ(data.GetSize(), 3) << "�R���X�g���N�^�ŏ����������ꍇ�C�T�C�Y��3�ł��D";
	}

	TEST(AsyncableDataTestVectorIntType, TestCanSetAndGetData)
	{
		AsyncableData<std::vector<int>> data;	//������

		std::vector<int> sample = { 1, 2, 3 };

		data.SetData(sample);
		EXPECT_EQ(data.GetData(), sample);

		sample = { 4, 5, 6 };
		data.SetData(sample);
		EXPECT_EQ(data.GetData(), sample);
	}

	TEST(AsyncableDataTestVectorIntType, TestCanPushBackAndGetData)
	{
		AsyncableData<std::vector<int>> data;	//������

		std::vector<int> sample = { 1, 2, 3 };

		data.SetData(sample);
		EXPECT_EQ(data.GetData(), sample);

		data.PushBack(4);
		sample.push_back(4);
		EXPECT_EQ(data.GetData(), sample);

		data.PushBack(5);
		sample.push_back(5);
		EXPECT_EQ(data.GetData(), sample);
	}

	TEST(AsyncableDataTestVectorIntType, TestCanClearAndGetData)
	{
		AsyncableData<std::vector<int>> data;	//������

		std::vector<int> sample = { 1, 2, 3 };

		data.SetData(sample);
		EXPECT_EQ(data.GetData(), sample);

		data.Clean();
		sample.clear();
		EXPECT_EQ(data.GetData(), sample);
	}

	TEST(AsyncableDataTestVectorIntType, GetUpdateCountTestCaseOfSetData)
	{
		AsyncableData<std::vector<int>> data;	//������
		EXPECT_EQ(data.GetUpdateCount(), 0) << "���������ɂ͍X�V�񐔂�0�ł��D";

		std::vector<int> sample = { 1, 2, 3 };

		data.SetData(sample);
		EXPECT_EQ(data.GetUpdateCount(), 1) << "�l���X�V�����ꍇ1�������܂��D";

		sample = { 4, 5, 6 };
		data.SetData(sample);
		EXPECT_EQ(data.GetUpdateCount(), 2) << "�l���X�V�����ꍇ1�������܂��D";
	}

	TEST(AsyncableDataTestVectorIntType, GetUpdateCountTestCaseOfGetData)
	{
		AsyncableData<std::vector<int>> data;	//������
		EXPECT_EQ(data.GetUpdateCount(), 0) << "���������ɂ͍X�V�񐔂�0�ł��D";

		std::vector<int> sample = { 1, 2, 3 };

		data.SetData(sample);
		EXPECT_EQ(data.GetUpdateCount(), 1) << "�l���X�V�����ꍇ1�������܂��D";

		data.GetData();
		EXPECT_EQ(data.GetUpdateCount(), 1) << "�l���擾���������ł͍X�V�񐔂͑����܂���D";

		data.GetData();
		EXPECT_EQ(data.GetUpdateCount(), 1) << "�l���擾���������ł͍X�V�񐔂͑����܂���D";
	}

	TEST(AsyncableDataTestVectorIntType, GetUpdateCountTestCaseOfPushData)
	{
		AsyncableData<std::vector<int>> data;	//������
		EXPECT_EQ(data.GetUpdateCount(), 0) << "���������ɂ͍X�V�񐔂�0�ł��D";

		std::vector<int> sample = { 1, 2, 3 };

		data.SetData(sample);
		EXPECT_EQ(data.GetUpdateCount(), 1) << "�l���X�V�����ꍇ1�������܂��D";

		data.PushBack(4);
		EXPECT_EQ(data.GetUpdateCount(), 2) << "�l���X�V�����ꍇ1�������܂��D";
	}

	TEST(AsyncableDataTestVectorIntType, GetUpdateCountTestCaseOfClear)
	{
		AsyncableData<std::vector<int>> data;	//������
		EXPECT_EQ(data.GetUpdateCount(), 0) << "���������ɂ͍X�V�񐔂�0�ł��D";

		std::vector<int> sample = { 1, 2, 3 };

		data.SetData(sample);
		EXPECT_EQ(data.GetUpdateCount(), 1) << "�l���X�V�����ꍇ1�������܂��D";

		data.Clean();
		EXPECT_EQ(data.GetUpdateCount(), 2) << "�l���X�V�����ꍇ1�������܂��D";
	}

	TEST(AsyncableDataTestVectorIntType, GetUpdateCountTestCaseOfGetSize)
	{
		AsyncableData<std::vector<int>> data;	//������
		EXPECT_EQ(data.GetUpdateCount(), 0) << "���������ɂ͍X�V�񐔂�0�ł��D";

		std::vector<int> sample = { 1, 2, 3 };

		data.SetData(sample);
		EXPECT_EQ(data.GetUpdateCount(), 1) << "�l���X�V�����ꍇ1�������܂��D";

		data.GetSize();
		EXPECT_EQ(data.GetUpdateCount(), 1) << "�l���X�V�����ꍇ1�������܂��D";
	}
}