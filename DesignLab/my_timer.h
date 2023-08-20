#pragma once
#include <chrono>


//! @class MyTimer
//! @date 2023/08/06
//! @author ���J��
//! @brief ���Ԍv���p�̃N���X�D
class MyTimer final
{
public:

	MyTimer() = default;

	//! @brief �v���J�n�ɂ��̊֐����ĂԁD���Z�b�g�����Ȃǂ��ĂԕK�v�Ȃ��ŁC�ēx�v���ł���D
	void start();

	//! @brief �v���I���ɂ��̊֐����ĂԁD
	void end();

	//! @brief �v�����ʂ�b�Ŏ擾�Dstart()��end()���Ăяo���Ă��炱�̊֐����ĂԁD
	//! @return double �v������(�b)
	double getSecond() const;

	//! @brief �v�����ʂ��~���b�Ŏ擾�Dstart()��end()���Ăяo���Ă��炱�̊֐����ĂԁD
	//! @return double �v������(�~���b)
	double getMilliSecond() const;

	//! @biref �v�����ʂ��}�C�N���b�Ŏ擾�Dstart()��end()���Ăяo���Ă��炱�̊֐����ĂԁD
	//! @return double  �v������(�}�C�N���b)
	double getMicroSecond() const;

private:


	std::chrono::system_clock::time_point m_start_time;		//����J�n����

	std::chrono::system_clock::time_point m_end_time;		//����I������	
};


//! @file my_timer.h
//! @date 2023/08/06
//! @author ���J��
//! @brief ����̎��Ԍv���N���X�D
//! @n �s�� : @lineinfo