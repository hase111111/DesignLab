#include "designlab_cmdio.h"

#include <cstdlib>
#include <iostream>
#include <string>


namespace
{
	// �o�͐����C���̒l�����̃��b�Z�[�W�̏o�͍͂s���Ȃ�
	// �Ȃ��C���̒l�͂��̃t�@�C������̂݃A�N�Z�X�\�ȃO���[�o���ϐ�
	OutputDetail output_limit = OutputDetail::kSystem;

	// false�̏ꍇ�C�o�͂��s��Ȃ�(�V�X�e�����b�Z�[�W�͏���)
	bool do_output = true;
}


namespace designlab
{
	namespace cmdio
	{
		void SetOutputLimit(const OutputDetail limit)
		{
			::output_limit = limit;
		}

		void SetDoOutput(bool do_output)
		{
			::do_output = do_output;
		}

		void Output(const std::string& str, const OutputDetail detail, const bool wait_cin)
		{
			// �o�͂������Ă���@���@�o�͂��镶����̏ڍׂ��ݒ�t�@�C���ŋ�����Ă���ꍇ�@�܂���
			// �o�͂������Ă��Ȃ��@���@�o�͂��镶����̏ڍׂ��V�X�e�����b�Z�[�W�̏ꍇ

			if ((detail <= ::output_limit && do_output) || (detail == OutputDetail::kSystem && !do_output))
			{
				std::cout << str;

				if (wait_cin)
				{
					std::cout << std::flush;
				}

				std::cout << "\n";
			}
		}

		void OutputNewLine(const int num, const OutputDetail detail)
		{
			if (num < 0) { return; }

			for (int i = 0; i < num; i++)
			{
				Output("", detail);
			}
		}

		void OutputHorizontalLine(const bool double_line, const OutputDetail detail)
		{
			std::string str;

			for (int i = 0; i < HORIZONTAL_LINE_LENGTH; i++)
			{
				if (double_line)
				{
					str += "=";

				}
				else
				{
					str += "-";
				}
			}

			Output(str, detail);
		}

		void OutputCenter(const std::string& str, const OutputDetail detail)
		{
			std::string space;

			for (int i = 0; i < (HORIZONTAL_LINE_LENGTH - str.length()) / 2; i++)
			{
				space += " ";
			}

			Output(space + str, detail);
		}

		void OutputRight(const std::string& str, const OutputDetail detail)
		{
			std::string space;

			for (int i = 0; i < HORIZONTAL_LINE_LENGTH - str.length(); i++)
			{
				space += " ";
			}

			Output(space + str, detail);
		}

		void OutputTitle()
		{
			OutputNewLine();
			OutputHorizontalLine(true, OutputDetail::kSystem);
			OutputNewLine();
			OutputCenter("DesignLab", OutputDetail::kSystem);
			OutputNewLine();
			OutputRight("Created by DesignLab", OutputDetail::kSystem);
			OutputRight("All rights reserved", OutputDetail::kSystem);
			OutputNewLine();
			OutputHorizontalLine(true, OutputDetail::kSystem);
			OutputNewLine();
		}

		void OutputGraphViewerTitle()
		{
			OutputNewLine();
			OutputHorizontalLine(true, OutputDetail::kSystem);
			OutputNewLine();
			OutputCenter("GraphViewer", OutputDetail::kSystem);
			OutputNewLine();
			OutputHorizontalLine(true, OutputDetail::kSystem);
			OutputNewLine();
		}


		void WaitAnyKey(const std::string& str)
		{
			Output(str, OutputDetail::kSystem, true);

			//�����L�[�������܂őҋ@
			system("PAUSE");
		}


		int InputInt(const int min, const int max, const int default_num, const std::string& str)
		{
			Output(str + " (" + std::to_string(min) + " ~ " + std::to_string(max) + ") : ", OutputDetail::kSystem, true);

			std::string input_str;
			std::cin >> input_str;

			int res = default_num;

			try
			{
				res = std::stoi(input_str);

				if (res < min || res > max)
				{
					res = default_num;
				}
			}
			catch (...)
			{
				res = default_num;
			}

			return res;
		}

		bool InputYesNo(const std::string& str)
		{
			Output(str, OutputDetail::kSystem, true);

			while (true)
			{
				std::string input_str;
				std::cin >> input_str;


				if (input_str == "y" || input_str == "yes" || input_str == "Y" || input_str == "Yes" || input_str == "YES")
				{
					return true;
				}
				else if (input_str == "n" || input_str == "no" || input_str == "N" || input_str == "No" || input_str == "NO")
				{
					return false;
				}

				Output("���͂��ꂽ�l�u" + input_str + "�v�͕]���ł��܂���Dy / n�œ��͂��Ă��������D", OutputDetail::kSystem, true);
			}

		}

		EBootMode SelectBootMode()
		{
			Output("�N�����[�h��I�����Ă�������", OutputDetail::kSystem);
			Output("0: �V�~�����[�V����", OutputDetail::kSystem);
			Output("1: �O���t�r���[���[", OutputDetail::kSystem);
			Output("2: �\���e�X�g", OutputDetail::kSystem);
			Output("3: ���ʂ̊m�F", OutputDetail::kSystem);
			Output("other: �f�t�H���g�̃��[�h ( " + std::to_string(EBootMode::SIMULATION) + " )", OutputDetail::kSystem);
			OutputNewLine();

			int input = InputInt(0, static_cast<int>(EBootMode::RESULT_VIEWER), 0);

			if (input == 0)
			{
				return EBootMode::SIMULATION;
			}
			else if (input == 1)
			{
				return EBootMode::VIEWER;
			}
			else if (input == 2)
			{
				return EBootMode::DISPLAY_TEST;
			}
			else if (input == 3)
			{
				return EBootMode::RESULT_VIEWER;
			}
			else
			{
				return EBootMode::SIMULATION;
			}
		}

	}	// namespace cmdio

}	// namespace designlab