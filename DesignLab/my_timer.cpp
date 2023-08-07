#include "my_timer.h"


void MyTimer::start()
{
	m_start_time = std::chrono::system_clock::now();
}


void MyTimer::end()
{
	m_end_time = std::chrono::system_clock::now();
}


double MyTimer::getSecond() const
{
	auto dur = m_end_time - m_start_time;        // 要した時間を計算

	auto sec = std::chrono::duration_cast<std::chrono::seconds>(dur).count();	// 秒に変換

	return static_cast<double>(sec);
}


double MyTimer::getMilliSecond() const
{
	auto dur = m_end_time - m_start_time;        // 要した時間を計算

	auto msec = std::chrono::duration_cast<std::chrono::milliseconds>(dur).count();		// ミリ秒に変換

	return static_cast<double>(msec);
}


double MyTimer::getMicroSecond() const
{
	auto dur = m_end_time - m_start_time;        // 要した時間を計算

	auto usec = std::chrono::duration_cast<std::chrono::microseconds>(dur).count();		// マイクロ秒に変換	

	return static_cast<double>(usec);
}
