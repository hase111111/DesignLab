//! @file cmdio_util.h
//! @brief �R�}���h���C���ɕ������o�͂���֐��D

#ifndef DESIGNLAB_CMDIO_UTIL_H_
#define DESIGNLAB_CMDIO_UTIL_H_

#include <string>

#include "output_detail.h"
#include "boot_mode.h"


namespace designlab
{
	//! @namespace designlab::cmdio
	//! @brief �R�}���h���C���ɕ������o�͂���֐����܂Ƃ߂����O��ԁDcommand line input/output �̗��D
	//! @details �R�}���h���C���ɕ������o�͂���֐����܂Ƃ߂����O��ԁD
	//! @n �ݒ�t�@�C���ŏo�͂̋������Ă��Ȃ��ꍇ�́C�o�͂��Ȃ��@�\�������������č쐬�����D
	//! @n ���ʓI�ɁC�R�}���h���C���ɏo�͂���֐������ׂĂ��̖��O��Ԃɓ����Ă��適
	namespace cmdio
	{
		constexpr int kHorizontalLineLength = 100; //!< �������̒����D


		//! @brief �o�͂��郁�b�Z�[�W���ǂ��܂ŋ����邩��ݒ肷��֐��D
		//! @n �Ⴆ�� kError �ɐݒ肷��ƁCkError �����̏o��( kInfo �Ƃ� kDebug �Ƃ�)�͂���Ȃ��D
		//! @n �t�� kDebug �ɐݒ肷��ƁC���ׂĂ̏o�͂������D
		//! @n 1�x�Ăяo������C�v���O�����I���܂Őݒ�͗L���ƂȂ�D
		//! @param [in] limit �o�͂��郁�b�Z�[�W���ǂ��܂ŋ����邩
		void SetOutputLimit(OutputDetail limit);

		//! @brief ���������o�͂����邩��ݒ肷��֐��D
		//! @n false�ɐݒ肵�Ă� �V�X�e�����b�Z�[�W�͏o�͂����D
		//! @param [in] do_output �o�͂����邩�ǂ���
		void SetDoOutput(bool do_output);


		//! @brief �R�}���h���C���ɕ������o�͂���֐��D
		//! @n SetOutputLimit() �Őݒ肵���o�͂̋��͈͓��ł���Ώo�͂����D
		//! @n �K��SetOutputLimit()���Ăяo���Ă���g�����ƁD
		//! @param [in] str �o�͂��镶����
		//! @param [in] detail �o�͂��镶����̏ڍ� (�f�t�H���g�ł�kSystem)
		//! @param [in] wait_cin cin��҂��ǂ��� (�f�t�H���g�ł�false)�C
		//! @n cin�Ƃ̓�����؂��Ă��邽�߁Ccin��҂��Ȃ��Əo�͏������������Ȃ邱�Ƃ�����D
		void Output(const std::string& str, OutputDetail detail = OutputDetail::kSystem, bool wait_cin = false);

		//! @brief �R�}���h���C���ŉ��s������֐��D
		//! @param [in] num ���s����� (�f�t�H���g�ł�1��)
		//! @n 0�ȉ��̒l������Ɖ������Ȃ��D
		//! @param [in] detail �o�͂���ۂ̗D��x (�f�t�H���g�ł�kSystem)
		void OutputNewLine(int num = 1, OutputDetail detail = OutputDetail::kSystem);

		//! @brief �R�}���h���C���ɐ��������o�͂���֐��D
		//! @param [in] double_line ��d���ɂ��邩�ǂ��� (�f�t�H���g�ł�false)
		//! @param [in] detail �o�͂���ۂ̗D��x (�f�t�H���g�ł�kSystem)
		void OutputHorizontalLine(bool double_line = false, OutputDetail detail = OutputDetail::kSystem);

		//! @brief �����ɕ������o�͂���֐�
		//! @param [in] str �o�͂��镶����
		//! @param [in] detail �o�͂���ۂ̗D��x (�f�t�H���g�ł�kSystem)
		void OutputCenter(const std::string& str, OutputDetail detail = OutputDetail::kSystem);

		//! @brief �E�[�ɕ������o�͂���֐�
		//! @param [in] str �o�͂��镶����
		//! @param [in] detail �o�͂���ۂ̗D��x (�f�t�H���g�ł�kSystem)
		void OutputRight(const std::string& str, OutputDetail detail = OutputDetail::kSystem);

		//! @brief �R�}���h���C���ɂ��̃\�t�g�̃^�C�g�����o�͂���֐��D
		//! @param [in] str �o�͂��镶����
		//! @param [in] output_copy_right �R�s�[���C�g���o�͂��邩�ǂ��� (�f�t�H���g�ł�false)
		void OutputTitle(const std::string& title_name,bool output_copy_right = false);


		//! @brief ���͑҂�������֐��D
		//! @param [in] str ���͑҂�������ۂɏo�͂��镶����
		void WaitAnyKey(const std::string& str = "���͑҂��CEnter�L�[�������Ă��������D");

		//! @brief ��������͂���֐��D
		//! @param [in] min ���͂��鐮���̍ŏ��l
		//! @param [in] max ���͂��鐮���̍ő�l
		//! @param [in] default_num �f�t�H���g�œ��͂��鐮��
		//! @param [in] str ���͑҂�������ۂɏo�͂��镶����
		//! @return int ���͂��ꂽ����
		int InputInt(int min, int max, int default_num, const std::string& str = "��������͂��Ă��������D");

		//! @brief yes��no����͂���֐��D�Ԃ�l��yes�Ȃ�true�Cno�Ȃ�false���󂯎��D
		//! @param [in] str ���͑҂�������ۂɏo�͂��镶����
		//! @return bool yes�Ȃ��true�Cno�Ȃ��false
		bool InputYesNo(const std::string& str = "��낵���ł����H");

		//! @brief ���̃A�v���̋N�����[�h��I������֐��D
		//! @return BootMode �I�������N�����[�h
		BootMode SelectBootMode();

	}	// namespace dl_cio

}	// namespace designlab


#endif	// DESIGNLAB_CMDIO_UTIL_H_