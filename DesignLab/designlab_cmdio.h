#pragma once

#include <string>

#include "node.h"
#include "application_setting_recorder.h"
#include "output_priority.h"



//! @namespace dl_cio
//! @date 2023/08/08
//! @author ���J��
//! @brief �R�}���h���C���ɕ������o�͂���֐����܂Ƃ߂����O��ԁDdesignlab (dl) command line input/output (cio)�̗��D
namespace dl_cio
{
	//! @brief �R�}���h���C���ɕ������o�͂���֐��D
	//! @param [in] setting �ݒ�t�@�C���̏����L�^�����\���̂̃|�C���^
	//! @param [in] str �o�͂��镶����
	//! @param [in] priority �o�͂���ۂ̗D��x (�f�t�H���g�ł�SYSTEM)
	//! @param [in] wait_cin cin��҂��ǂ��� (�f�t�H���g�ł�false)�Ccin�Ƃ̓�����؂��Ă��邽�߁Ccin��҂��Ȃ��Əo�͏������������Ȃ邱�Ƃ�����D
	//! @param [in] display_func_name �֐������o�͂��邩�ǂ��� (�f�t�H���g�ł�false)
	inline void output(const SApplicationSettingRecorder* setting, const std::string str,
		const EOutputPriority priority = EOutputPriority::SYSTEM, const bool wait_cin = false, const bool display_func_name = false)
	{
		if ((priority <= (*setting).cmd_permission && (*setting).cmd_output) || (priority == EOutputPriority::SYSTEM && !(*setting).cmd_output))
		{
			if (display_func_name)
			{
				std::cout << __func__ << " : ";
			}

			std::cout << str;

			if (wait_cin)
			{
				std::cout << std::flush;
			}

			std::cout << std::endl;
		}
	}


	//! @brief �R�}���h���C���ŉ��s������֐��D
	//! @param [in] setting �ݒ�t�@�C���̏����L�^�����\���̂̃|�C���^
	//! @param [in] num ���s����� (�f�t�H���g�ł�1��)
	//! @param [in] priority �o�͂���ۂ̗D��x (�f�t�H���g�ł�SYSTEM)
	void outputNewLine(const SApplicationSettingRecorder* setting, const int num = 1, const EOutputPriority priority = EOutputPriority::SYSTEM);


	//! @brief �R�}���h���C���Ƀ^�C�g�����o�͂���֐��D
	//! @param [in] setting �ݒ�t�@�C���̏����L�^�����\���̂̃|�C���^
	void outputTitle(const SApplicationSettingRecorder* setting);


	//! @brief �R�}���h���C���ɐ��������o�͂���֐��D
	//! @param [in] setting �ݒ�t�@�C���̏����L�^�����\���̂̃|�C���^
	//! @param [in] double_line ��d���ɂ��邩�ǂ��� (�f�t�H���g�ł�false)
	//! @param [in] priority �o�͂���ۂ̗D��x (�f�t�H���g�ł�SYSTEM)
	void outputHorizontalLine(const SApplicationSettingRecorder* setting, const bool double_line = false, const EOutputPriority priority = EOutputPriority::SYSTEM);


	//! @brief ���͑҂�������֐��D
	//! @param [in] str ���͑҂�������ۂɏo�͂��镶����
	void waitAnyKey(const SApplicationSettingRecorder* setting, const std::string str = "���͑҂��CEnter�L�[�������Ă��������D",
		const EOutputPriority priority = EOutputPriority::SYSTEM);


	//! @brief ��������͂���֐��D
	//! @param [in] setting �ݒ�t�@�C���̏����L�^�����\���̂̃|�C���^
	//! @param [in] min ���͂��鐮���̍ŏ��l
	//! @param [in] max ���͂��鐮���̍ő�l
	//! @param [in] default_num �f�t�H���g�œ��͂��鐮��
	//! @param [in] str ���͑҂�������ۂɏo�͂��镶����
	//! @return int ���͂��ꂽ����
	int inputInt(const SApplicationSettingRecorder* setting, const int min, const int max, const int default_num, const std::string str = "��������͂��Ă��������D");


	EBootMode selectBootMode(const SApplicationSettingRecorder* setting);

}	//namespace dl_cio



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

	////�V�~�����[�V�����̌��ʂ��R�}���h���C���ɏo�͂���
	//void outputSimulateResult(const int _loop_num, const SimulateResult& _res);

	static void setOutputPriority(const EOutputPriority priority) { m_output_priority = priority; }

	static EOutputPriority getOutputPriority() { return m_output_priority; }

private:

	//�p��̐����͏��� ( 1st�C2nd�C3rd�C4th�݂����Ȃ��)�����̂ŁC�󂯎�������l�ɏ�����t�����������Ԃ��֐��D
	std::string getOrdinalNumber(const int _num) const;

	static EOutputPriority m_output_priority;	//�R�}���h���C���ɕ������o�͂���ۂ̗D��x
};

//! @file designlab_cmd_io.h
//! @date 2023/08/08
//! @author ���J��
//! @brief �R�}���h���C���ɕ������o�͂��邽�߂̖��O��ԁD
//! @n �s�� : @lineinfo
