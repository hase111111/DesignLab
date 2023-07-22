#pragma once
#include <chrono>


class MyTimer final
{
public:

	MyTimer() = default;

	//! @brief �v���J�n
	void start()
	{
		m_start_time = std::chrono::system_clock::now();
	}

	//! @brief �v���I��
	void end()
	{
		m_end_time = std::chrono::system_clock::now();
	}

	//! @brief �v�����ʂ�b�Ŏ擾
	//! @return double �v������(�b)
	double getSecond() const
	{
		auto dur = m_end_time - m_start_time;        // �v�������Ԃ��v�Z
		auto sec = std::chrono::duration_cast<std::chrono::seconds>(dur).count();

		return (double)sec;
	}

	//! @brief �v�����ʂ��~���b�Ŏ擾
	//! @return double �v������(�~���b)
	double getMilliSecond() const
	{
		auto dur = m_end_time - m_start_time;        // �v�������Ԃ��v�Z
		auto msec = std::chrono::duration_cast<std::chrono::milliseconds>(dur).count();

		return (double)msec;
	}

	//! @biref �v�����ʂ��}�C�N���b�Ŏ擾
	//! @return double  �v������(�}�C�N���b)
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


//! @file MyTimer.h
//! @brief ���Ԍv���p�̃N���X�D
//! @date 2023/07/18
//! @auther ���J��

//! @class MyTimer
//! @brief ���Ԍv���p�̃N���X�D
//! @date 2023/07/18
//! @auther ���J��
