#pragma once
#include "SimulateResult.h"
#include "listFunc.h"
#include <string>

//�R�}���h���C���ɕ������o�͂���class 
//main.cpp �������Ⴒ���Ⴕ�Č��Â炢�̂ō���Ă݂�

class CmdIO final 
{
public:
	CmdIO() = default;
	~CmdIO() = default;

	void outputString(const std::string _str);

	//�m�[�h�̗l�q���R�}���h���C���ɏo�͂���
	void outputLNODE(const LNODE _node);

	//�V�~�����[�V�����̌��ʂ��R�}���h���C���ɏo�͂���
	void outputSimulateResult(const int _loop_num, const SimulateResult& _res);
};
