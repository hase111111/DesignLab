#include "cmdio_util.h"

#include <cstdlib>
#include <iostream>
#include <sstream>
#include <string>

#include "cassert_define.h"


namespace
{
	// ���̂悤�Ȗ��O�̂Ȃ����O��Ԃ𓽖����O��ԂƂ���

	// �������O��Ԃɂ��ꂽ�l�́C���̃t�@�C������̂݃A�N�Z�X�\�ȃO���[�o���ϐ��ƂȂ�
	// �A�N�Z�X����ꍇ�� :: ��擪�ɂ���

	// (�����܂ł���Ȃ�N���X�ɂ��Ƃ���悩��������)


	// �o�͐����C���̒l�����̃��b�Z�[�W�̏o�͍͂s���Ȃ�
	OutputDetail output_limit = OutputDetail::kSystem;

	// false�̏ꍇ�C�o�͂��s��Ȃ�(�V�X�e�����b�Z�[�W�͏���)
	bool do_output = true;

	// �����������ɍs�������ǂ���
	bool is_initialized = false;
}


namespace designlab
{
	namespace cmdio
	{
		void SetOutputLimit(const OutputDetail limit)
		{
			::output_limit = limit;

			if (!::is_initialized) 
			{
				//������L�q���Ă����Ǝ��s���x�������Ȃ�D���̂����printf���g�p�ł��Ȃ��D�ڂ���Reference���Q��
				std::cin.tie(&std::cout);
				std::ios_base::sync_with_stdio(true);

				::is_initialized = true;
			}
		}

		void SetDoOutput(const bool do_output_)
		{
			::do_output = do_output_;
		}


		void Output(const std::string& str, const OutputDetail detail)
		{
			assert(is_initialized);	// SetOutputLimit���Ă�ł���g�p���邱��.


			// �o�͂������Ă���@���@�o�͂��镶����̏ڍׂ��ݒ�t�@�C���ŋ�����Ă���ꍇ�@�܂���
			// �o�͂������Ă��Ȃ��@���@�o�͂��镶����̏ڍׂ��V�X�e�����b�Z�[�W�̏ꍇ

			if ((detail <= ::output_limit && do_output) || (detail == OutputDetail::kSystem && !do_output))
			{
				std::cout << str << std::endl;
			}
		}

		void OutputCenter(const std::string& str, const OutputDetail detail)
		{
			//���s���Ƃɕ���������o��
			std::stringstream ss(str);
			std::string line;

			while (std::getline(ss, line))
			{
				if (kHorizontalLineLength > line.length())
				{
					std::string space;

					for (int i = 0; i < (kHorizontalLineLength - line.length()) / 2; i++)
					{
						space += " ";
					}

					Output(space + line, detail);
				}
				else
				{
					Output(line, detail);
				}

			}
		}

		void OutputRight(const std::string& str, const OutputDetail detail)
		{
			//���s���Ƃɕ���������o��
			std::stringstream ss(str);
			std::string line;

			while (std::getline(ss, line))
			{
				if (kHorizontalLineLength > line.length())
				{
					std::string space;

					for (int i = 0; i < kHorizontalLineLength - line.length(); i++)
					{
						space += " ";
					}

					Output(space + line, detail);
				}
				else
				{
					Output(line, detail);
				}

			}
		}


		void OutputNewLine(const int num, const OutputDetail detail)
		{
			if (num <= 0) { return; }

			for (int i = 0; i < num; i++)
			{
				Output("", detail);
			}
		}

		void OutputHorizontalLine(const std::string& line_visual, const OutputDetail detail)
		{
			if (line_visual.size() != 1) { return; }

			std::string str;

			for (int i = 0; i < kHorizontalLineLength; i++)
			{
				str += line_visual;
			}

			Output(str, detail);
		}

		void OutputTitle(const std::string& title_name, bool output_copy_right)
		{
			OutputDetail detail = OutputDetail::kSystem;

			OutputNewLine(1, detail);
			OutputHorizontalLine("=", detail);
			OutputNewLine(1, detail);
			OutputCenter(title_name, detail);
			OutputNewLine(1, detail);

			if (output_copy_right) 
			{
				OutputRight("Coprright 2015 - 2023 ��ʑ�w �݌v�H�w������  ", detail);
				OutputNewLine(1, detail);
			}

			OutputHorizontalLine("=", detail);
			OutputNewLine(1, detail);
		}


		void WaitAnyKey(const std::string& str)
		{
			Output(str, OutputDetail::kSystem);

			//�����L�[�������܂őҋ@
			system("PAUSE");
		}

		int InputInt(const int min, const int max, const int default_num, const std::string& str)
		{
			assert(min <= max);	// min��max��菬�����D

			Output(str + " (" + std::to_string(min) + " �` " + std::to_string(max) + ") : ", OutputDetail::kSystem);

			std::string input_str;
			std::cout << std::flush;
			std::cin >> input_str;

			int res = default_num;

			try
			{
				res = std::stoi(input_str);	// ���͂��ꂽ�������int�^�ɕϊ�

				if (res < min || res > max)
				{
					Output(
						"���͂��ꂽ�l�u" + input_str + "�v�͔͈͊O�ł��D�f�t�H���g�̒l�C�u" + std::to_string(default_num) + "�v���g�p���܂��D",
						OutputDetail::kSystem
					);

					res = default_num;
				}
			}
			catch (...)
			{
				// stoi�ŗ�O�����������ꍇ�C�����ɏ��������

				Output(
					"���͂��ꂽ�l�u" + input_str + "�v�͕]���ł��܂���D�f�t�H���g�̒l�C�u" + std::to_string(default_num) + "�v���g�p���܂��D", 
					OutputDetail::kSystem
				);

				res = default_num;
			}

			return res;
		}

		bool InputYesNo(const std::string& str)
		{
			Output(str + " ( y / n ) ", OutputDetail::kSystem);

			while (true)
			{
				std::string input_str;
				std::cout << std::flush;
				std::cin >> input_str;


				if (input_str == "y" || input_str == "yes" || input_str == "Y" || input_str == "Yes" || input_str == "YES")
				{
					return true;
				}
				else if (input_str == "n" || input_str == "no" || input_str == "N" || input_str == "No" || input_str == "NO")
				{
					return false;
				}

				Output("���͂��ꂽ�l�u" + input_str + "�v�͕]���ł��܂���Dy / n�œ��͂��Ă��������D", OutputDetail::kSystem);
			}

		}

		BootMode SelectBootMode()
		{
			Output("�N�����[�h��I�����Ă�������", OutputDetail::kSystem);
			Output("0: �V�~�����[�V����", OutputDetail::kSystem);
			Output("1: �O���t�r���[���[", OutputDetail::kSystem);
			Output("2: �\���e�X�g", OutputDetail::kSystem);
			Output("3: ���ʂ̊m�F", OutputDetail::kSystem);
			Output("other: �f�t�H���g�̃��[�h ( " + std::to_string(BootMode::kSimulation) + " )", OutputDetail::kSystem);
			OutputNewLine(1,OutputDetail::kSystem);

			int input = InputInt(0, static_cast<int>(BootMode::kResultViewer), 0);

			if (input == 0)
			{
				return BootMode::kSimulation;
			}
			else if (input == 1)
			{
				return BootMode::kViewer;
			}
			else if (input == 2)
			{
				return BootMode::kDisplayTest;
			}
			else if (input == 3)
			{
				return BootMode::kResultViewer;
			}
			else
			{
				return BootMode::kSimulation;
			}
		}

	}	// namespace cmdio

}	// namespace designlab