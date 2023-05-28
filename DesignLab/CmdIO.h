#pragma once
#include "SimulateResult.h"
#include "Node.h"
#include <string>

//�R�}���h���C���ɕ������o�͂���N���X�Dmain�ɒ���std::cout�������Ă������̂����C���₷����������Ӗ������˂Ă��̃N���X�ɕ����Ă������D
class CmdIO final 
{
public:
	CmdIO() = default;
	~CmdIO() = default;

	void outputString(const std::string _str);

	//�m�[�h�̗l�q���R�}���h���C���ɏo�͂���D_num�ɐ����l������ƁC���݂̓��삪������ڂ��o�͂��Ă����D
	void outputNode(const SNode& _node, const int _num = -1) const;

	//�O���t�T�������s�������̃G���[���b�Z�[�W���o�͂���D
	void outputErrorMessageInGraphSearch(const std::string _err_mes) const;

	//�O���t�T���J�n���̃��b�Z�[�W���o�͂���D�����ɂāC����ڂ̃V�~�����[�V�����Ȃ̂����擾����D
	void outputGraphSearchStaretMessage(const int _simu_num) const;

	//�V�~�����[�V�����̌��ʂ��R�}���h���C���ɏo�͂���
	void outputSimulateResult(const int _loop_num, const SimulateResult& _res);

private:

	//�p��̐����͏��� ( 1st�C2nd�C3rd�C4th�݂����Ȃ��)�����̂ŁC�󂯎�������l�ɏ�����t�����������Ԃ��֐��D
	std::string getOrdinalNumber(const int _num) const;
};
