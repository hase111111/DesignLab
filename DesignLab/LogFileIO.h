#pragma once
#include <string>
#include <fstream>
#include "listFunc.h"
#include "SimulateResult.h"

// File Input Output �܂�t�@�C�����o�͂̂���

class LogFileIO final
{
public:
	LogFileIO();
	~LogFileIO() = default;

	//Log�t�@�C�����쐬����D���s�����false��Ԃ�
	bool openLogFile();

	//Log�t�@�C�������
	void closeLogFile();

	//Log�t�@�C���ɐ��l����������
	void addLogStringWithInt(const int _data);

	//Log�t�@�C���Ƀm�[�h�̏�����������
	void addLogStringWithNode(const int num, const LNODE& node_log);

	//Log�t�@�C���ɕ��������������
	void addLogString(const std::string _str);

	//Log�t�@�C���ɃV�~�����[�V�����̌��ʂ���������
	void addLogStringSimulation(const int _loop_num, const SimulateResult& _res);

private:
	const std::string LOG_NAME = "WalkingPatternLog";	//�o�͂���t�@�C����
	const std::string LOG_EXTENSION = ".csv";			//�g���q

	int m_log_num = 1;
	std::string m_file_name;
	std::ofstream m_all_log;
};