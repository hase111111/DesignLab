#include "stopwatch.h"

#include "designlab_math_util.h"


namespace sc = ::std::chrono;
namespace dlm = ::designlab::math_util;


Stopwatch::Stopwatch() : 
	start_time_(sc::system_clock::now()),
	end_time_(sc::system_clock::now())
{
}

void Stopwatch::Start()
{
	start_time_ = sc::system_clock::now();
}


void Stopwatch::End()
{
	end_time_ = sc::system_clock::now();
}


double Stopwatch::GetElapsedSeconds() const
{
	const auto dur = end_time_ - start_time_;        // �v�������Ԃ��v�Z

	const long long sec = sc::duration_cast<sc::seconds>(dur).count();			// �b�ɕϊ�
	const long long msec = sc::duration_cast<sc::milliseconds>(dur).count();	// �~���b�ɕϊ�

	return static_cast<double>(sec) + static_cast<double>(msec) / 1000.0;
}


double Stopwatch::GetElapsedMilliSecond() const
{
	const auto dur = end_time_ - start_time_;        // �v�������Ԃ��v�Z

	const long long msec = sc::duration_cast<sc::milliseconds>(dur).count();	// �~���b�ɕϊ�
	const long long usec = sc::duration_cast<sc::microseconds>(dur).count();	// �}�C�N���b�ɕϊ�

	return static_cast<double>(msec) + static_cast<double>(usec) / 1000.0;
}


double Stopwatch::GetElapsedMicroSecond() const
{
	const auto dur = end_time_ - start_time_;        // �v�������Ԃ��v�Z

	const auto usec = sc::duration_cast<sc::microseconds>(dur).count();		// �}�C�N���b�ɕϊ�	

	return static_cast<double>(usec);
}


std::string Stopwatch::GetElapsedSecondsString() const
{
	return dlm::ConvertDoubleToString(GetElapsedSeconds()) + " [s]";
}

std::string Stopwatch::GetElapsedMilliSecondString() const
{
	return dlm::ConvertDoubleToString(GetElapsedMilliSecond()) + " [ms]";
}

std::string Stopwatch::GetElapsedMicroSecondString() const
{
	return dlm::ConvertDoubleToString(GetElapsedMicroSecond()) + "[��s]";
}


std::string Stopwatch::GetNowTimeString() const
{
	// ���݂̓�����YYYY/MM/DD HH:MM�`���̕�����Ŏ擾����
	auto now = std::chrono::system_clock::now();
	auto now_time = std::chrono::system_clock::to_time_t(now);
	std::tm now_tm;
	localtime_s(&now_tm, &now_time);
	char now_time_str[64];
	strftime(now_time_str, sizeof(now_time_str), "%Y%m%d_%H%M_%S", &now_tm);
	return std::string(now_time_str);
}