#pragma once
#include <string>
#include <fstream>
#include "Node.h"
#include "SimulateResult.h"

class LogFileIO final
{
public:
	LogFileIO();
	~LogFileIO() = default;

	//! @brief Log�t�@�C�����쐬����D���s�����false��Ԃ�
	//! @return bool �t�@�C���쐬�ɐ���������true�C���s������false
	bool openLogFile();

	//! @brief Log�t�@�C�������
	void closeLogFile();

	//! @brief Log�t�@�C���ɐ��l����������
	//! @param [in] _data �������ސ��l
	void addLogStringWithInt(const int _data);

	//! @brief Log�t�@�C���Ƀm�[�h�̏�����������
	//! @param [in] num �m�[�h�̔ԍ�
	//! @param [in] node_log �������ރm�[�h�̏��
	void addLogStringWithNode(const int num, const SNode& node_log);

	//! @brief Log�t�@�C���ɕ��������������
	//! @param [in] _str �������ޕ�����
	void addLogString(const std::string _str);

	////! @brief Log�t�@�C���ɃV�~�����[�V�����̌��ʂ���������
	////! @param [in] _loop_num ���[�v��
	////! @param [in] _res �V�~�����[�V�����̌���
	//void addLogStringSimulation(const int _loop_num, const SimulateResult& _res);

private:
	const std::string LOG_NAME = "WalkingPatternLog";	//�o�͂���t�@�C����
	const std::string LOG_EXTENSION = ".csv";			//�g���q

	int m_log_num = 1;
	std::string m_file_name;
	std::ofstream m_all_log;
};

//! @file LogFileIO.h
//! @brief ���O�t�@�C���̓��o�͂��s���N���X�D
//! @date 2023/06/17
//! @author ���J��

//! @class LogFileIO
//! @brief ���O�t�@�C���̓��o�͂��s���N���X�D
//! @date 2023/06/17
//! @author ���J��
