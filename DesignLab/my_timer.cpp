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
	auto dur = m_end_time - m_start_time;        // �v�������Ԃ��v�Z

	auto sec = std::chrono::duration_cast<std::chrono::seconds>(dur).count();	// �b�ɕϊ�

	return static_cast<double>(sec);
}


double MyTimer::getMilliSecond() const
{
	auto dur = m_end_time - m_start_time;        // �v�������Ԃ��v�Z

	auto msec = std::chrono::duration_cast<std::chrono::milliseconds>(dur).count();		// �~���b�ɕϊ�

	return static_cast<double>(msec);
}


double MyTimer::getMicroSecond() const
{
	auto dur = m_end_time - m_start_time;        // �v�������Ԃ��v�Z

	auto usec = std::chrono::duration_cast<std::chrono::microseconds>(dur).count();		// �}�C�N���b�ɕϊ�	

	return static_cast<double>(usec);
}
