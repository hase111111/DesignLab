#include "pch.h"

#include "../DesignLab/cmdio_util.h"
#include "../DesignLab/cmdio_util.cpp"


namespace dlio = ::designlab::cmdio;


namespace 
{
	//�e�X�g�t�B�N�X�`��
	class CmdioUtilTest : public ::testing::Test
	{
	protected:

		void SetUp() override
		{
			dlio::SetOutputLimit(OutputDetail::kDebug);
			dlio::SetDoOutput(true);
		}

		void TearDown() override
		{
			//�e�X�g�̌㏈��
		}
	};
}


namespace designlab::test::common::commandio
{
	TEST_F(CmdioUtilTest, OutputTestCaseOfDebug)
	{
		// cout�̏o�͂��L���v�`������D
		// https://gongo.hatenablog.com/entry/20110516/1305548031 �Q�lURL�D
		// �Ǝv�������C����ܐ�������ĂȂ����ۂ�? https://blog.emattsan.org/entry/20120613/1339584605

		testing::internal::CaptureStdout();
		dlio::Output("test", OutputDetail::kSystem);
		EXPECT_EQ(testing::internal::GetCapturedStdout(), "test\n");

		testing::internal::CaptureStdout();
		dlio::Output("test", OutputDetail::kError);
		EXPECT_EQ(testing::internal::GetCapturedStdout(), "test\n");

		testing::internal::CaptureStdout();
		dlio::Output("test", OutputDetail::kWarning);
		EXPECT_EQ(testing::internal::GetCapturedStdout(), "test\n");

		testing::internal::CaptureStdout();
		dlio::Output("test", OutputDetail::kInfo);
		EXPECT_EQ(testing::internal::GetCapturedStdout(), "test\n");

		testing::internal::CaptureStdout();
		dlio::Output("test", OutputDetail::kDebug);
		EXPECT_EQ(testing::internal::GetCapturedStdout(), "test\n");
	}

	TEST_F(CmdioUtilTest, OutputTestCaseOfInfo)
	{
		dlio::SetOutputLimit(OutputDetail::kInfo);

		testing::internal::CaptureStdout();
		dlio::Output("test", OutputDetail::kSystem);
		EXPECT_EQ(testing::internal::GetCapturedStdout(), "test\n");

		testing::internal::CaptureStdout();
		dlio::Output("test", OutputDetail::kError);
		EXPECT_EQ(testing::internal::GetCapturedStdout(), "test\n");

		testing::internal::CaptureStdout();
		dlio::Output("test", OutputDetail::kWarning);
		EXPECT_EQ(testing::internal::GetCapturedStdout(), "test\n");

		testing::internal::CaptureStdout();
		dlio::Output("test", OutputDetail::kInfo);
		EXPECT_EQ(testing::internal::GetCapturedStdout(), "test\n");

		testing::internal::CaptureStdout();
		dlio::Output("test", OutputDetail::kDebug);
		EXPECT_EQ(testing::internal::GetCapturedStdout(), "");
	}

	TEST_F(CmdioUtilTest, OutputTestCaseOfWarning)
	{
		dlio::SetOutputLimit(OutputDetail::kWarning);

		testing::internal::CaptureStdout();
		dlio::Output("test", OutputDetail::kSystem);
		EXPECT_EQ(testing::internal::GetCapturedStdout(), "test\n");

		testing::internal::CaptureStdout();
		dlio::Output("test", OutputDetail::kError);
		EXPECT_EQ(testing::internal::GetCapturedStdout(), "test\n");

		testing::internal::CaptureStdout();
		dlio::Output("test", OutputDetail::kWarning);
		EXPECT_EQ(testing::internal::GetCapturedStdout(), "test\n");

		testing::internal::CaptureStdout();
		dlio::Output("test", OutputDetail::kInfo);
		EXPECT_EQ(testing::internal::GetCapturedStdout(), "");
		
		testing::internal::CaptureStdout();
		dlio::Output("test", OutputDetail::kDebug);
		EXPECT_EQ(testing::internal::GetCapturedStdout(), "");
	}

	TEST_F(CmdioUtilTest, OutputTestCaseOfError)
	{
		dlio::SetOutputLimit(OutputDetail::kError);

		testing::internal::CaptureStdout();
		dlio::Output("test", OutputDetail::kSystem);
		EXPECT_EQ(testing::internal::GetCapturedStdout(), "test\n");

		testing::internal::CaptureStdout();
		dlio::Output("test", OutputDetail::kError);
		EXPECT_EQ(testing::internal::GetCapturedStdout(), "test\n");

		testing::internal::CaptureStdout();
		dlio::Output("test", OutputDetail::kWarning);
		EXPECT_EQ(testing::internal::GetCapturedStdout(), "");
		
		testing::internal::CaptureStdout();
		dlio::Output("test", OutputDetail::kInfo);
		EXPECT_EQ(testing::internal::GetCapturedStdout(), "");
		
		testing::internal::CaptureStdout();
		dlio::Output("test", OutputDetail::kDebug);
		EXPECT_EQ(testing::internal::GetCapturedStdout(), "");
	}

	TEST_F(CmdioUtilTest, OutputTestCaseOfSystem)
	{
		dlio::SetOutputLimit(OutputDetail::kSystem);

		testing::internal::CaptureStdout();
		dlio::Output("test", OutputDetail::kSystem);
		EXPECT_EQ(testing::internal::GetCapturedStdout(), "test\n");

		testing::internal::CaptureStdout();
		dlio::Output("test", OutputDetail::kError);
		EXPECT_EQ(testing::internal::GetCapturedStdout(), "");

		testing::internal::CaptureStdout();
		dlio::Output("test", OutputDetail::kWarning);
		EXPECT_EQ(testing::internal::GetCapturedStdout(), "");

		testing::internal::CaptureStdout();
		dlio::Output("test", OutputDetail::kInfo);
		EXPECT_EQ(testing::internal::GetCapturedStdout(), "");

		testing::internal::CaptureStdout();
		dlio::Output("test", OutputDetail::kDebug);
		EXPECT_EQ(testing::internal::GetCapturedStdout(), "");
	}

	TEST_F(CmdioUtilTest, OutputTestCaseOfDonotOutput)
	{
		dlio::SetDoOutput(false);

		testing::internal::CaptureStdout();
		dlio::Output("test", OutputDetail::kSystem);
		EXPECT_EQ(testing::internal::GetCapturedStdout(), "test\n");

		testing::internal::CaptureStdout();
		dlio::Output("test", OutputDetail::kError);
		EXPECT_EQ(testing::internal::GetCapturedStdout(), "");

		testing::internal::CaptureStdout();
		dlio::Output("test", OutputDetail::kWarning);
		EXPECT_EQ(testing::internal::GetCapturedStdout(), "");

		testing::internal::CaptureStdout();
		dlio::Output("test", OutputDetail::kInfo);
		EXPECT_EQ(testing::internal::GetCapturedStdout(), "");

		testing::internal::CaptureStdout();
		dlio::Output("test", OutputDetail::kDebug);
		EXPECT_EQ(testing::internal::GetCapturedStdout(), "");
	}
	
}
