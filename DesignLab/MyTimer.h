#pragma once
#include <chrono>

//���Ԍv���p�̃N���X
class MyTimer final
{
public:

	MyTimer() = default;

	//�v���J�n
	void start()
	{
		m_start_time = std::chrono::system_clock::now();
	}

	//�v���I��
	void end()
	{
		m_end_time = std::chrono::system_clock::now();
	}

	//�v�����ʂ��~���b�Ŏ擾
	double getMilliSecond() const
	{
		auto dur = m_end_time - m_start_time;        // �v�������Ԃ��v�Z
		auto msec = std::chrono::duration_cast<std::chrono::milliseconds>(dur).count();

		return (double)msec;
	}

	//�v�����ʂ�b�Ŏ擾
	double getSecond() const
	{
		auto dur = m_end_time - m_start_time;        // �v�������Ԃ��v�Z
		auto sec = std::chrono::duration_cast<std::chrono::seconds>(dur).count();

		return (double)sec;
	}

	//�v�����ʂ��}�C�N���b�Ŏ擾	
	double getMicroSecond() const
	{
		auto dur = m_end_time - m_start_time;        // �v�������Ԃ��v�Z
		auto usec = std::chrono::duration_cast<std::chrono::microseconds>(dur).count();

		return (double)usec;
	}

private:
	//���Ԍv���p�̕ϐ�
	std::chrono::system_clock::time_point m_start_time;
	std::chrono::system_clock::time_point m_end_time;
};