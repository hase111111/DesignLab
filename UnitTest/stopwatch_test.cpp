#include "pch.h"

#include <thread>

#include "../DesignLab/stopwatch.h"
#include "../DesignLab/stopwatch.cpp"	//�ǂ���1�̃t�@�C���ŁCcpp�t�@�C�����C���N���[�h����K�v������


// ���Ԍv���N���X�̃e�X�g�D���x���ǂ̒��x�o��̂��킩��Ȃ��̂ŁC�����Ńe�X�g����Ӗ�������̂��͕�����Ȃ�...
// �ǂ�����΂悢���m���Ă�l�͋����Ă���D
// ���m�Ɏ��Ԃ��v���ł��邩�ł͂Ȃ��C�֐��̋@�\�����������̂݃e�X�g����D


namespace designlab::test::common 
{
	TEST(StopwatchTest, ConstructorTestCanTimerStart) 
	{
		Stopwatch sw;

		std::this_thread::sleep_for(std::chrono::milliseconds(10));	//0.01�b�҂�

		sw.End();

		EXPECT_NE(sw.GetElapsedMilliSecond(), 0.0) << "�R���X�g���N�^���Ă΂ꂽ���_�Ń^�C�}�[�͊J�n���܂��D";
	}

	TEST(StopwatchTest, EndTestCanTimerStop)
	{
		Stopwatch sw;

		sw.Start();

		std::this_thread::sleep_for(std::chrono::milliseconds(10));	//0.01�b�҂�

		sw.End();

		const double first_elapsed_time = sw.GetElapsedMilliSecond();

		std::this_thread::sleep_for(std::chrono::milliseconds(10));	//0.01�b�҂�

		const double second_elapsed_time = sw.GetElapsedMilliSecond();

		EXPECT_EQ(first_elapsed_time, second_elapsed_time) << "End()���Ă΂ꂽ���_�Ń^�C�}�[�͒�~���܂��D";
	}

	TEST(StopwatchTest, StartTestCanTimerRestart)
	{
		Stopwatch sw;
		Stopwatch long_sw;

		sw.Start();
		long_sw.Start();

		std::this_thread::sleep_for(std::chrono::milliseconds(10));	//0.01�b�҂�

		sw.End();

		const double first_elapsed_time = sw.GetElapsedMilliSecond();

		sw.Start();

		std::this_thread::sleep_for(std::chrono::milliseconds(10));	//0.01�b�҂�

		sw.End();
		long_sw.End();

		const double second_elapsed_time = sw.GetElapsedMilliSecond();

		const double long_elapsed_time = long_sw.GetElapsedMilliSecond();

		EXPECT_LT(first_elapsed_time, long_elapsed_time) << "Start()���Ă΂ꂽ���_�Ń^�C�}�[�̓��X�^�[�g���܂��D";	// less than <
		EXPECT_LT(second_elapsed_time, long_elapsed_time) << "Start()���Ă΂ꂽ���_�Ń^�C�}�[�̓��X�^�[�g���܂��D";
		EXPECT_LT(0.f, first_elapsed_time) << "Start()���Ă΂ꂽ���_�Ń^�C�}�[�̓��X�^�[�g���܂��D";
		EXPECT_LT(0.f, second_elapsed_time) << "Start()���Ă΂ꂽ���_�Ń^�C�}�[�̓��X�^�[�g���܂��D";

		//�v�����x�̖��ňȉ��̃e�X�g�͎��s���邩�������邩���s����
		//EXPECT_EQ(first_elapsed_time, second_elapsed_time) << "Start()���Ă΂ꂽ���_�Ń^�C�}�[�̓��X�^�[�g���܂��D";
	}

	TEST(StopwatchTest, TestTimeUnitCheck)
	{
		// �P�ʂ������Ă��邩���m�F����e�X�g

		Stopwatch sw;

		sw.Start();

		std::this_thread::sleep_for(std::chrono::milliseconds(10));	//0.01�b�҂�

		sw.End();

		const double elapsed_seconds = sw.GetElapsedSeconds();
		const double elapsed_milliseconds = sw.GetElapsedMilliSecond();
		const double elapsed_microseconds = sw.GetElapsedMicroSecond();

		EXPECT_EQ(static_cast<int>(elapsed_seconds), static_cast<int>(elapsed_milliseconds / 1000.0)) << "1 [sec] = 1000 [msec]";
		EXPECT_EQ(static_cast<int>(elapsed_milliseconds), static_cast<int>(elapsed_microseconds / 1000.0)) << "1 [msec] = 1000 [usec]";
	}
}