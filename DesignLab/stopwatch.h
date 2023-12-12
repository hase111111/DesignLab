//! @file stopwatch.h
//! @brief ����̎��Ԍv���N���X�D

#ifndef DESIGNLAB_STOPWATCH_H_
#define DESIGNLAB_STOPWATCH_H_

#include <chrono>
#include <string>


//! @class Stopwatch
//! @brief ���Ԍv���p�̃N���X�D
//! @details
//! Start�֐����Ăяo���Ă���End�֐����Ăяo���܂ł̌o�ߎ��Ԃ��v������D
//! @n ���̓R���X�g���N�^�ɂČ��ݎ����ŏ���������̂ŁCStart�֐����Ă΂��Ƃ��v���J�n�ł���D
class Stopwatch final
{
public:

	//! @brief �R���X�g���N�^
	//! @n ���ݎ����ŏ���������D
	Stopwatch();

	//! @brief �v���J�n�ɂ��̊֐����ĂԁD
	//! @n ���Z�b�g�����Ȃǂ��ĂԕK�v�Ȃ��ŁC�ēx�v���ł���D
	void Start();

	//! @brief �v���I�����ɂ��̊֐����ĂԁD
	void End();

	//! @brief �o�ߎ��Ԃ�b�Ŏ擾�D
	//! @n start()��end()���Ăяo���Ă��炱�̊֐����ĂԁD
	//! @return double �v������(�b)�D
	double GetElapsedSeconds() const;

	//! @brief �v�����ʂ��~���b�Ŏ擾�D
	//! @n start()��end()���Ăяo���Ă��炱�̊֐����ĂԁD
	//! @return double �v������(�~���b)�D
	double GetElapsedMilliSecond() const;

	//! @brief �v�����ʂ��}�C�N���b�Ŏ擾�D
	//! @n start()��end()���Ăяo���Ă��炱�̊֐����ĂԁD
	//! @n (���̃��x���̃I�[�_�[�Ő��x�o����̂�...?)�D
	//! @return double  �v������(�}�C�N���b)�D
	double GetElapsedMicroSecond() const;


	//! @brief �o�ߎ��Ԃ�b�ŕ\������������擾����D
	//! @return std::string �o�ߎ��ԁD
	std::string GetElapsedSecondsString() const;

	//! @brief �o�ߎ��Ԃ��~���b�ŕ\������������擾����D
	//! @return std::string �o�ߎ��ԁD
	std::string GetElapsedMilliSecondString() const;

	//! @brief �o�ߎ��Ԃ��}�C�N���b�ŕ\������������擾����D
	//! @return std::string �o�ߎ��ԁD
	std::string GetElapsedMicroSecondString() const;


	//! @brief ���݂̓�����YYYY/MM/DD HH:MM�`���̕�����Ŏ擾����D
	//! @return std::string ���݂̓����D
	std::string GetNowTimeString() const;

private:

	std::chrono::system_clock::time_point start_time_;		//!< ����J�n����

	std::chrono::system_clock::time_point end_time_;		//!< ����I������	
};


#endif	// DESIGNLAB_STOPWATCH_H_