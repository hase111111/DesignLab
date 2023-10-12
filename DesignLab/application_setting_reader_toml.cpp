#include "application_setting_reader_toml.h"

#include <iostream>
#include <fstream>
#include <filesystem>

#include <magic_enum.hpp>

#include "output_detail.h"


std::shared_ptr<ApplicationSettingRecorder> ApplicationSettingReaderToml::Read()
{
	std::cout << "�ݒ�t�@�C��" << kSettingFileName << "��ǂݍ��݂܂�\n\n";

	//�t�@�C����T���C���݂��Ȃ�������f�t�H���g�̐ݒ���o�͂��ďI���Cfsystem��C++17����C���s�ł��Ȃ��ꍇ�͐ݒ���������Ă݂Ă�������
	if (!std::filesystem::is_regular_file(kSettingFileName))
	{
		std::cout << "�ݒ�t�@�C����������܂���ł����D�f�t�H���g�̐ݒ�t�@�C�����o�͂��܂��D\n";
		OutputDefaultSettingFile();
		return std::make_shared<ApplicationSettingRecorder>();
	}

	std::cout << "�ݒ�t�@�C����������܂����D�ǂݍ��݂��J�n�������܂�\n\n";

	//�t�@�C����ǂݍ���
	toml::value data;

	try
	{
		std::ifstream ifs(kSettingFileName, std::ios::binary);		//�o�C�i�����[�h�œǂݍ���

		data = toml::parse(ifs, kSettingFileName);					//�t�@�C�����p�[�X(�ǂ݂���&���)����
	}
	catch (toml::syntax_error err)
	{
		std::cout << "�ݒ�t�@�C���̓ǂݍ��݂Ɏ��s���܂����D�f�t�H���g�̐ݒ�t�@�C�����o�͂��܂��D\n";
		std::cout << err.what() << std::endl;

		OutputDefaultSettingFile();

		return std::make_shared<ApplicationSettingRecorder>();
	}


	std::cout << "�ݒ�t�@�C���̓ǂݍ��݂ɐ������܂����D�t�@�C����title�L�[���m�F���܂�\n\n";


	if (toml::get<std::string>(data.at(ApplicationSettingTomlKey::kFileTitle.key)) != ApplicationSettingTomlKey::kFileTitleValue)
	{
		//�t�@�C���̃^�C�g������v���Ȃ��ꍇ�̓f�t�H���g�̐ݒ���o�͂��ďI��
		std::cout << "�ݒ�t�@�C���̃^�C�g������v���܂���ł����D�f�t�H���g�̐ݒ�t�@�C�����o�͂��܂��D\n";
		
		OutputDefaultSettingFile();

		return std::make_shared<ApplicationSettingRecorder>();
	}


	std::cout << "�ݒ�t�@�C���̃^�C�g������v���܂����D�ݒ�t�@�C���̓ǂݍ��݂𑱍s���܂�\n\n";

	std::shared_ptr<ApplicationSettingRecorder> result = std::make_shared<ApplicationSettingRecorder>();

	try
	{
		ReadVersionSetting(data, result);

		ReadBootModeSetting(data, result);

		ReadDisplaySetting(data, result);
	}
	catch (...)
	{
		//�ݒ�t�@�C���̓ǂݍ��݂Ɏ��s�����ꍇ�̓f�t�H���g�̐ݒ���o�͂��ďI��
		std::cout << "�ݒ�t�@�C���̓ǂݍ��݂̓r���ŃG���[���������܂����D�ǂݍ��߂Ȃ������ݒ�̓f�t�H���g�̒l���g�p���܂��D\n";
		std::cout << "�f�t�H���g�̐ݒ�t�@�C�����o�͂��܂��D\n";

		OutputDefaultSettingFile();
		
		return std::make_shared<ApplicationSettingRecorder>();
	}


	std::cout << "�ݒ�t�@�C���̓ǂݍ��݂��������܂����D\n";

	return std::move(result);
}



void ApplicationSettingReaderToml::OutputDefaultSettingFile()
{
	const ApplicationSettingRecorder kDefaultSetting;

	std::string res_str;	//�o�͂��镶����


	//�t�@�C���̃^�C�g��
	{
		res_str += u8"# If this file is garbled, the problem is most likely due to character encoding.\n";
		res_str += u8"# This file is written in utf - 8, and can be read by installing VS Code and configuring it to automatically detect the character encoding.\n\n";
		res_str += u8"# This file is written in the TOML format. Just google it and you'll find easy to understand information on the wiki, so try looking it up.\n";
		res_str += u8"# This file is a file for describing program settings.\n";
		res_str += u8"# This file can also be edited with a text editor such as Notepad.\n";
		res_str += u8"# If you want to change the simulation conditions, try changing them from here.\n";
		res_str += u8"# Lines starting with a sharp are comments. They do not affect the program.\n";
		res_str += u8"# Following description is written in Japanese. \n\n";
		res_str += u8"# TOML�Ƃ����`���ŋL�q���Ă��܂��Dwiki�ŃO�O�邾���ł��킩��₷����񂪏o��̂Œ��ׂĂ݂Ă��������D\n";
		res_str += u8"# ���̃t�@�C���́C�v���O�����̐ݒ���L�q���邽�߂̃t�@�C���ł��D\n";
		res_str += u8"# �������Ȃǂ̃e�L�X�g�G�f�B�^�ł����̃t�@�C���͕ҏW���邱�Ƃ��ł��܂��D\n";
		res_str += u8"# �V�~�����[�V����������ύX�������ꍇ�́C��������ύX���s���悤�ɂ��Ă݂Ă��������D\n";
		res_str += u8"# ���̂悤�ɃV���[�v�Ŏn�܂�s�̓R�����g�ł��D�v���O�����ɉe����^���Ȃ����߁C��������Ɏg�����Ƃ��ł��܂��D\n";
		res_str += u8"\n\n\n";

		toml::value title_data{
			{ ApplicationSettingTomlKey::kFileTitle.key, ApplicationSettingTomlKey::kFileTitleValue }
		};
		title_data.comments().push_back(ApplicationSettingTomlKey::kFileTitle.description);

		res_str += toml::format(title_data, 0);
		res_str += u8"\n";
	}


	//�o�[�W����
	{
		toml::basic_value<toml::preserve_comments> version_data{
			{
				ApplicationSettingTomlKey::kVersionTable.table_name,
				{
					{ ApplicationSettingTomlKey::kVersionMajor.key, kDefaultSetting.version_major},
					{ ApplicationSettingTomlKey::kVersionMinor.key, kDefaultSetting.version_minor },
					{ ApplicationSettingTomlKey::kVersionPatch.key, kDefaultSetting.version_patch }
				}
			}
		};

		version_data.at(ApplicationSettingTomlKey::kVersionTable.table_name).comments().push_back(ApplicationSettingTomlKey::kVersionTable.description);
		version_data.at(ApplicationSettingTomlKey::kVersionTable.table_name).at(ApplicationSettingTomlKey::kVersionMajor.key).comments().push_back(ApplicationSettingTomlKey::kVersionMajor.description);
		version_data.at(ApplicationSettingTomlKey::kVersionTable.table_name).at(ApplicationSettingTomlKey::kVersionMinor.key).comments().push_back(ApplicationSettingTomlKey::kVersionMinor.description);
		version_data.at(ApplicationSettingTomlKey::kVersionTable.table_name).at(ApplicationSettingTomlKey::kVersionPatch.key).comments().push_back(ApplicationSettingTomlKey::kVersionPatch.description);

		res_str += toml::format(version_data, 0);
	}

	//���[�h
	{
		toml::basic_value<toml::preserve_comments> mode_data{
			{
				ApplicationSettingTomlKey::kMoveTable.table_name,
				{
					{ ApplicationSettingTomlKey::kAskAboutBootMode.key, kDefaultSetting.ask_about_modes },
					{ ApplicationSettingTomlKey::kDefaultMode.key, magic_enum::enum_name(kDefaultSetting.default_mode) },
					{ ApplicationSettingTomlKey::kDoStepExecution.key, kDefaultSetting.do_step_execution },
					{ ApplicationSettingTomlKey::kDoStepEexcutionEachGait.key, kDefaultSetting.do_step_execution_each_gait}
				}
			}
		};

		mode_data.at(ApplicationSettingTomlKey::kMoveTable.table_name).comments().push_back(ApplicationSettingTomlKey::kMoveTable.description);
		mode_data.at(ApplicationSettingTomlKey::kMoveTable.table_name).at(ApplicationSettingTomlKey::kAskAboutBootMode.key).comments().push_back(ApplicationSettingTomlKey::kAskAboutBootMode.description);
		mode_data.at(ApplicationSettingTomlKey::kMoveTable.table_name).at(ApplicationSettingTomlKey::kDefaultMode.key).comments().push_back(ApplicationSettingTomlKey::kDefaultMode.description);
		mode_data.at(ApplicationSettingTomlKey::kMoveTable.table_name).at(ApplicationSettingTomlKey::kDoStepExecution.key).comments().push_back(ApplicationSettingTomlKey::kDoStepExecution.description);
		mode_data.at(ApplicationSettingTomlKey::kMoveTable.table_name).at(ApplicationSettingTomlKey::kDoStepEexcutionEachGait.key).comments().push_back(ApplicationSettingTomlKey::kDoStepEexcutionEachGait.description);

		res_str += toml::format(mode_data, 0);
	}

	// �\��
	{
		toml::basic_value<toml::preserve_comments> display_data{
			{
				ApplicationSettingTomlKey::kDisplayTable.table_name,
				{
					{ ApplicationSettingTomlKey::kOutputCmd.key, kDefaultSetting.cmd_output},
					{ ApplicationSettingTomlKey::kCmdPermission.key, magic_enum::enum_name(kDefaultSetting.cmd_permission) },
					{ ApplicationSettingTomlKey::kDisplayGui.key, kDefaultSetting.gui_display },
					{ ApplicationSettingTomlKey::kGuiDisplayQuality.key, kDefaultSetting.gui_display_quality },
					{ ApplicationSettingTomlKey::kWindowSizeX.key,kDefaultSetting.window_size_x },
					{ ApplicationSettingTomlKey::kWindowSizeY.key,kDefaultSetting.window_size_y },
					{ ApplicationSettingTomlKey::kWindowFps.key,kDefaultSetting.window_fps }
				}
			}
		};

		display_data.at(ApplicationSettingTomlKey::kDisplayTable.table_name).comments().push_back(ApplicationSettingTomlKey::kDisplayTable.description);
		display_data.at(ApplicationSettingTomlKey::kDisplayTable.table_name).at(ApplicationSettingTomlKey::kOutputCmd.key).comments().push_back(ApplicationSettingTomlKey::kOutputCmd.description);
		display_data.at(ApplicationSettingTomlKey::kDisplayTable.table_name).at(ApplicationSettingTomlKey::kCmdPermission.key).comments().push_back(ApplicationSettingTomlKey::kCmdPermission.description);
		display_data.at(ApplicationSettingTomlKey::kDisplayTable.table_name).at(ApplicationSettingTomlKey::kDisplayGui.key).comments().push_back(ApplicationSettingTomlKey::kDisplayGui.description);
		display_data.at(ApplicationSettingTomlKey::kDisplayTable.table_name).at(ApplicationSettingTomlKey::kGuiDisplayQuality.key).comments().push_back(ApplicationSettingTomlKey::kGuiDisplayQuality.description);
		display_data.at(ApplicationSettingTomlKey::kDisplayTable.table_name).at(ApplicationSettingTomlKey::kWindowSizeX.key).comments().push_back(ApplicationSettingTomlKey::kWindowSizeX.description);
		display_data.at(ApplicationSettingTomlKey::kDisplayTable.table_name).at(ApplicationSettingTomlKey::kWindowSizeY.key).comments().push_back(ApplicationSettingTomlKey::kWindowSizeY.description);
		display_data.at(ApplicationSettingTomlKey::kDisplayTable.table_name).at(ApplicationSettingTomlKey::kWindowFps.key).comments().push_back(ApplicationSettingTomlKey::kWindowFps.description);

		res_str += toml::format(display_data, 0);
	}

	std::ofstream ofs;
	ofs.open(kSettingFileName);

	// �t�@�C�����J���Ȃ������牽�����Ȃ�
	if (!ofs)
	{
		std::cout << "�f�t�H���g�̐ݒ�t�@�C���̏o�͂Ɏ��s���܂����D\n";
		return;
	}

	ofs.write(res_str.c_str(), res_str.length());	// �t�@�C���ɏ�������

	ofs.close();	// �t�@�C�������

	return;
}


void ApplicationSettingReaderToml::ReadVersionSetting(const toml::value& value, std::shared_ptr<ApplicationSettingRecorder>& recorder)
{
	std::cout << std::endl;

	if (value.contains(ApplicationSettingTomlKey::kVersionTable.table_name))
	{
		std::cout << "�Z�o�[�W�����ݒ��ǂݍ��݂܂��D\n";

		// �o�[�W�����ݒ��ǂݍ���
		if (value.at(ApplicationSettingTomlKey::kVersionTable.table_name).contains(ApplicationSettingTomlKey::kVersionMajor.key))
		{
			recorder->version_major = (int)value.at(ApplicationSettingTomlKey::kVersionTable.table_name).at(ApplicationSettingTomlKey::kVersionMajor.key).as_integer();
			std::cout << "�Z���W���[�o�[�W������ǂݍ��݂܂����Dvalue = " << recorder->version_major << "\n";
		}
		else
		{
			std::cout << "�~���W���[�o�[�W������������܂���ł����D\n";
		}

		if (value.at(ApplicationSettingTomlKey::kVersionTable.table_name).contains(ApplicationSettingTomlKey::kVersionMinor.key))
		{
			recorder->version_minor = (int)value.at(ApplicationSettingTomlKey::kVersionTable.table_name).at(ApplicationSettingTomlKey::kVersionMinor.key).as_integer();
			std::cout << "�Z�}�C�i�[�o�[�W������ǂݍ��݂܂����Dvalue = " << recorder->version_minor << "\n";
		}
		else
		{
			std::cout << "�~�}�C�i�[�o�[�W������������܂���ł����D\n";
		}

		if (value.at(ApplicationSettingTomlKey::kVersionTable.table_name).contains(ApplicationSettingTomlKey::kVersionPatch.key))
		{
			recorder->version_patch = (int)value.at(ApplicationSettingTomlKey::kVersionTable.table_name).at(ApplicationSettingTomlKey::kVersionPatch.key).as_integer();
			std::cout << "�Z�p�b�`�o�[�W������ǂݍ��݂܂����Dvalue = " << recorder->version_patch << "\n";
		}
		else
		{
			std::cout << "�~�p�b�`�o�[�W������������܂���ł����D\n";
		}
	}
	else
	{
		std::cout << "�~�o�[�W�����ݒ肪������܂���ł����D\n";
	}
}


void ApplicationSettingReaderToml::ReadBootModeSetting(const toml::value& value, std::shared_ptr<ApplicationSettingRecorder>& recorder)
{
	std::cout << std::endl;

	if (value.contains(ApplicationSettingTomlKey::kMoveTable.table_name))
	{
		std::cout << "�Z�N�����[�h�ݒ��ǂݍ��݂܂��D\n";

		if (value.at(ApplicationSettingTomlKey::kMoveTable.table_name).contains(ApplicationSettingTomlKey::kAskAboutBootMode.key))
		{
			recorder->ask_about_modes = value.at(ApplicationSettingTomlKey::kMoveTable.table_name).at(ApplicationSettingTomlKey::kAskAboutBootMode.key).as_boolean();
			std::cout << "�Z�N�����[�h�I���̊m�F�t���O��ǂݍ��݂܂����Dvalue = " << recorder->ask_about_modes << "\n";
		}
		else
		{
			std::cout << "�~�N�����[�h�I���̊m�F�t���O��������܂���ł����D\n";
		}

		if (value.at(ApplicationSettingTomlKey::kMoveTable.table_name).contains(ApplicationSettingTomlKey::kDefaultMode.key))
		{
			std::string read_value = value.at(ApplicationSettingTomlKey::kMoveTable.table_name).at(ApplicationSettingTomlKey::kDefaultMode.key).as_string();
			recorder->default_mode = magic_enum::enum_cast<BootMode>(read_value).value();
			std::cout << "�Z�f�t�H���g�̋N�����[�h��ǂݍ��݂܂����Dvalue = " << magic_enum::enum_name(recorder->default_mode) << "\n";
		}
		else
		{
			std::cout << "�~�f�t�H���g�̋N�����[�h��������܂���ł����D\n";
		}

		if (value.at(ApplicationSettingTomlKey::kMoveTable.table_name).contains(ApplicationSettingTomlKey::kDoStepExecution.key))
		{
			recorder->do_step_execution = value.at(ApplicationSettingTomlKey::kMoveTable.table_name).at(ApplicationSettingTomlKey::kDoStepExecution.key).as_boolean();
			std::cout << "�Z�X�e�b�v���s�t���O��ǂݍ��݂܂����Dvalue = " << std::boolalpha << recorder->do_step_execution << "\n";
		}
		else
		{
			std::cout << "�~�X�e�b�v���s�t���O��������܂���ł����D\n";
		}

		if (value.at(ApplicationSettingTomlKey::kMoveTable.table_name).contains(ApplicationSettingTomlKey::kDoStepEexcutionEachGait.key))
		{
			recorder->do_step_execution_each_gait = value.at(ApplicationSettingTomlKey::kMoveTable.table_name).at(ApplicationSettingTomlKey::kDoStepEexcutionEachGait.key).as_boolean();
			std::cout << "�Z�X�e�b�v���s�t���O(�e���e)��ǂݍ��݂܂����Dvalue = " << std::boolalpha << recorder->do_step_execution_each_gait << "\n";
		}
		else
		{
			std::cout << "�~�X�e�b�v���s�t���O(�e���e)��������܂���ł����D\n";
		}

	}
	else
	{
		std::cout << "�~�N�����[�h�ݒ肪������܂���ł����D\n";
	}
}


void ApplicationSettingReaderToml::ReadDisplaySetting(const toml::value& value, std::shared_ptr<ApplicationSettingRecorder>& recorder)
{
	std::cout << "\n";

	if (value.contains(ApplicationSettingTomlKey::kDisplayTable.table_name))
	{
		std::cout << "�Z�\���ݒ��ǂݍ��݂܂��D" << "\n";

		if (value.at(ApplicationSettingTomlKey::kDisplayTable.table_name).contains(ApplicationSettingTomlKey::kOutputCmd.key))
		{
			recorder->cmd_output = value.at(ApplicationSettingTomlKey::kDisplayTable.table_name).at(ApplicationSettingTomlKey::kOutputCmd.key).as_boolean();
			std::cout << "�Z�R�}���h�o�͂̃t���O��ǂݍ��݂܂����Dvalue = " << std::boolalpha << recorder->cmd_output << "\n";
		}
		else
		{
			std::cout << "�~�R�}���h�o�͂̃t���O��������܂���ł����D\n";
		}

		if (value.at(ApplicationSettingTomlKey::kDisplayTable.table_name).contains(ApplicationSettingTomlKey::kCmdPermission.key))
		{
			std::string read_str = value.at(ApplicationSettingTomlKey::kDisplayTable.table_name).at(ApplicationSettingTomlKey::kCmdPermission.key).as_string();
			recorder->cmd_permission = magic_enum::enum_cast<OutputDetail>(read_str).value();
			std::cout << "�Z�R�}���h�\�������̏���ǂݍ��݂܂����Dvalue = " << magic_enum::enum_name(recorder->cmd_permission) << "\n";
		}
		else
		{
			std::cout << "�~�R�}���h�\�������̏�񂪌�����܂���ł����D\n";
		}

		if (value.at(ApplicationSettingTomlKey::kDisplayTable.table_name).contains(ApplicationSettingTomlKey::kDisplayGui.key))
		{
			recorder->gui_display = value.at(ApplicationSettingTomlKey::kDisplayTable.table_name).at(ApplicationSettingTomlKey::kDisplayGui.key).as_boolean();
			std::cout << "�ZGUI�\���̃t���O��ǂݍ��݂܂����Dvalue = " << std::boolalpha << recorder->gui_display << "\n";
		}
		else
		{
			std::cout << "�~GUI�\���̃t���O��������܂���ł����D\n";
		}

		if (value.at(ApplicationSettingTomlKey::kGuiDisplayQuality.table_name).contains(ApplicationSettingTomlKey::kGuiDisplayQuality.key))
		{
			recorder->gui_display_quality = value.at(ApplicationSettingTomlKey::kGuiDisplayQuality.table_name).at(ApplicationSettingTomlKey::kGuiDisplayQuality.key).as_string();
			std::cout << "�ZGUI�\���i����ǂݍ��݂܂����Dvalue = " << recorder->gui_display_quality << "\n";
		}
		else
		{
			std::cout << "�~GUI�\���i����������܂���ł����D\n";
		}

		if (value.at(ApplicationSettingTomlKey::kDisplayTable.table_name).contains(ApplicationSettingTomlKey::kWindowSizeX.key))
		{
			recorder->window_size_x = (int)value.at(ApplicationSettingTomlKey::kDisplayTable.table_name).at(ApplicationSettingTomlKey::kWindowSizeX.key).as_integer();
			std::cout << "�Z�E�B���h�E��X�T�C�Y(����)�̒l��ǂݍ��݂܂����Dvalue = " << recorder->window_size_x << "\n";
		}
		else
		{
			std::cout << "�~�E�B���h�E��X�T�C�Y(����)�̒l��������܂���ł����D\n";
		}

		if (value.at(ApplicationSettingTomlKey::kDisplayTable.table_name).contains(ApplicationSettingTomlKey::kWindowSizeY.key))
		{
			recorder->window_size_y = (int)value.at(ApplicationSettingTomlKey::kDisplayTable.table_name).at(ApplicationSettingTomlKey::kWindowSizeY.key).as_integer();
			std::cout << "�Z�E�B���h�E��Y�T�C�Y(�c��)�̒l��ǂݍ��݂܂����Dvalue = " << recorder->window_size_y << "\n";
		}
		else
		{
			std::cout << "�~�E�B���h�E��Y�T�C�Y(�c��)�̒l��������܂���ł����D\n";
		}

		if (value.at(ApplicationSettingTomlKey::kDisplayTable.table_name).contains(ApplicationSettingTomlKey::kWindowFps.key))
		{
			recorder->window_fps = (int)value.at(ApplicationSettingTomlKey::kDisplayTable.table_name).at(ApplicationSettingTomlKey::kWindowFps.key).as_integer();
			std::cout << "�Z�E�B���h�E��FPS�̒l��ǂݍ��݂܂����Dvalue = " << recorder->window_fps << "\n";
		}
		else
		{
			std::cout << "�~�E�B���h�E��FPS�̒l��������܂���ł����D\n";
		}
	}
	else
	{
		std::cout << "�~�\���ݒ肪������܂���ł����D\n";
	}
}
