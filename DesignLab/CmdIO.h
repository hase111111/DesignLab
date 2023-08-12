#pragma once

#include <string>

#include "SimulateResult.h"
#include "Node.h"


//! @enum EOutputPriority
//! @date 2023/08/08
//! @author ���J��
//! @brief �R�}���h���C���ɕ������o�͂���ۂ̗D��x�D
enum class EOutputPriority
{
	ALWAYS,		//��ɏo�͂���
	FATAL,		//�v���I�ȃG���[
	ERROR_MES,		//�G���[
	WARNING,	//�x��
	INFO,		//���
};


//! @namespace dl_io
//! @date 2023/08/08
//! @author ���J��
//! @brief �R�}���h���C���ɕ������o�͂���֐����܂Ƃ߂����O��ԁD
namespace dl_io
{
	//! @brief �R�}���h���C���ɕ������o�͂���ۂ̗D��x��ݒ肷��֐��D
	//! @param [in] priority �o�͂���ۂ̗D��x
	void setOutputPriority(const EOutputPriority priority);

	//! @brief �R�}���h���C���ɕ������o�͂���֐��D
	//! @param [in] str �o�͂��镶����
	//! @param [in] priority �o�͂���ۂ̗D��x (�f�t�H���g�ł�ALWAYS)
	void output(const std::string str, const EOutputPriority priority = EOutputPriority::ALWAYS);

}	//namespace dl_io

//! @class CmdIO
//! @date 2023/06/17
//! @author ���J��
//! @brief �R�}���h���C���ɕ������o�͂���N���X�Dmain�ɒ���std::cout�������Ă������̂����C���₷����������Ӗ������˂Ă��̃N���X�ɕ����Ă������D
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

	static void setOutputPriority(const EOutputPriority priority) { m_output_priority = priority; }

	static EOutputPriority getOutputPriority() { return m_output_priority; }

private:

	//�p��̐����͏��� ( 1st�C2nd�C3rd�C4th�݂����Ȃ��)�����̂ŁC�󂯎�������l�ɏ�����t�����������Ԃ��֐��D
	std::string getOrdinalNumber(const int _num) const;

	static EOutputPriority m_output_priority;	//�R�}���h���C���ɕ������o�͂���ۂ̗D��x
};

//! @file cmd_io.h
//! @date 2023/08/08
//! @author ���J��
//! @brief �R�}���h���C���ɕ������o�͂��邽�߂̖��O��ԁD
//! @n �s�� : @lineinfo
