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

			std::cout << "\n";
		}
	}


	//! @brief �R�}���h���C���ŉ��s������֐��D
	//! @param [in] setting �ݒ�t�@�C���̏����L�^�����\���̂̃|�C���^
	//! @param [in] num ���s����� (�f�t�H���g�ł�1��)
	//! @param [in] priority �o�͂���ۂ̗D��x (�f�t�H���g�ł�SYSTEM)
	void outputNewLine(const SApplicationSettingRecorder* setting, const int num = 1, const EOutputPriority priority = EOutputPriority::SYSTEM);


	constexpr int HORIZONTAL_LINE_LENGTH = 70; //!< �������̒����D


	//! @brief �R�}���h���C���ɐ��������o�͂���֐��D
	//! @param [in] setting �ݒ�t�@�C���̏����L�^�����\���̂̃|�C���^
	//! @param [in] double_line ��d���ɂ��邩�ǂ��� (�f�t�H���g�ł�false)
	//! @param [in] priority �o�͂���ۂ̗D��x (�f�t�H���g�ł�SYSTEM)
	void outputHorizontalLine(const SApplicationSettingRecorder* setting, const bool double_line = false, const EOutputPriority priority = EOutputPriority::SYSTEM);


	//! @brief �����ɕ������o�͂���֐�
	//! @param [in] setting �ݒ�t�@�C���̏����L�^�����\���̂̃|�C���^
	//! @param [in] str �o�͂��镶����
	//! @param [in] priority �o�͂���ۂ̗D��x (�f�t�H���g�ł�SYSTEM)
	void outputCenter(const SApplicationSettingRecorder* setting, const std::string str, const EOutputPriority priority = EOutputPriority::SYSTEM);


	//! @brief �E�[�ɕ������o�͂���֐�
	//! @param [in] setting �ݒ�t�@�C���̏����L�^�����\���̂̃|�C���^
	//! @param [in] str �o�͂��镶����
	//! @param [in] priority �o�͂���ۂ̗D��x (�f�t�H���g�ł�SYSTEM)
	void outputRight(const SApplicationSettingRecorder* setting, const std::string str, const EOutputPriority priority = EOutputPriority::SYSTEM);


	//! @brief �R�}���h���C���Ƀ^�C�g�����o�͂���֐��D
	//! @param [in] setting �ݒ�t�@�C���̏����L�^�����\���̂̃|�C���^
	void outputTitle(const SApplicationSettingRecorder* setting);


	//! @brief �R�}���h���C����GraphViewer�̃^�C�g�����o�͂���֐��D
	//! @param [in] setting �ݒ�t�@�C���̏����L�^�����\���̂̃|�C���^
	void outputGraphViewerTitle(const SApplicationSettingRecorder* setting);


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


	//! @brief yes��no����͂���֐��D�Ԃ�l��yes�Ȃ�true�Cno�Ȃ�false���󂯎��D
	//! @param [in] setting �ݒ�t�@�C���̏����L�^�����\���̂̃|�C���^
	//! @param [in] str ���͑҂�������ۂɏo�͂��镶����
	//! @return bool yes�Ȃ��true�Cno�Ȃ��false
	bool inputYesNo(const SApplicationSettingRecorder* setting, const std::string str = "y / n�œ��͂��Ă��������D");


	EBootMode selectBootMode(const SApplicationSettingRecorder* setting);

}	//namespace dl_cio



//! @file designlab_cmd_io.h
//! @date 2023/08/08
//! @author ���J��
//! @brief �R�}���h���C���ɕ������o�͂��邽�߂̖��O��ԁD
//! @n �s�� : @lineinfo
