#include "application_setting_reader.h"

#include <iostream>
#include <fstream>
#include <filesystem>


void ApplicationSettingReader::read(SApplicationSettingRecorder* recorder)
{
	std::cout << "[" << __func__ << "]" << "�ݒ�t�@�C��" << SETTING_FILE_NAME << "��ǂݍ��݂܂�" << std::endl;
	std::cout << std::endl;


	//�t�@�C����T���C���݂��Ȃ�������f�t�H���g�̐ݒ���o�͂��ďI���Cfsystem��C++17����C���s�ł��Ȃ��ꍇ�͐ݒ���������Ă݂Ă�������
	if (!std::filesystem::is_regular_file(SETTING_FILE_NAME))
	{
		std::cout << "�ݒ�t�@�C����������܂���ł����D�f�t�H���g�̐ݒ�t�@�C�����o�͂��܂��D" << std::endl;
		outputDefaultSettingFile();
		return;
	}

	std::cout << "�ݒ�t�@�C����������܂����D�ǂݍ��݂��J�n�������܂�" << std::endl;
	std::cout << std::endl;


	//�t�@�C����ǂݍ���
	toml::value data;

	try
	{
		std::ifstream ifs(SETTING_FILE_NAME, std::ios::binary);		//�o�C�i�����[�h�œǂݍ���

		data = toml::parse(ifs, SETTING_FILE_NAME);					//�t�@�C�����p�[�X(�ǂ݂���)����
	}
	catch (toml::syntax_error e)
	{
		std::cout << "�ݒ�t�@�C���̓ǂݍ��݂Ɏ��s���܂����D�f�t�H���g�̐ݒ�t�@�C�����o�͂��܂��D" << std::endl;
		std::cout << e.what() << std::endl;
		outputDefaultSettingFile();
		return;
	}


	std::cout << "�ݒ�t�@�C���̓ǂݍ��݂ɐ������܂����D�t�@�C����title�L�[���m�F���܂�" << std::endl;
	std::cout << std::endl;


	if (toml::get<std::string>(data.at(ApplicationSettingKey::FILE_TITLE.key)) != ApplicationSettingKey::FILE_TITLE_VALUE)
	{
		//�t�@�C���̃^�C�g������v���Ȃ��ꍇ�̓f�t�H���g�̐ݒ���o�͂��ďI��
		std::cout << "�ݒ�t�@�C���̃^�C�g������v���܂���ł����D�f�t�H���g�̐ݒ�t�@�C�����o�͂��܂��D" << std::endl;
		outputDefaultSettingFile();
		return;
	}


	std::cout << "�ݒ�t�@�C���̃^�C�g������v���܂����D�ݒ�t�@�C���̓ǂݍ��݂𑱍s���܂�" << std::endl;
	std::cout << std::endl;


	try
	{
		readVersionSetting(data, recorder);

		readBootModeSetting(data, recorder);

		readDisplaySetting(data, recorder);
	}
	catch (...)
	{
		//�ݒ�t�@�C���̓ǂݍ��݂Ɏ��s�����ꍇ�̓f�t�H���g�̐ݒ���o�͂��ďI��
		std::cout << "�ݒ�t�@�C���̓ǂݍ��݂̓r���ŃG���[���������܂����D�ǂݍ��߂Ȃ������ݒ�̓f�t�H���g�̒l���g�p���܂��D" << std::endl;
		std::cout << "�f�t�H���g�̐ݒ�t�@�C�����o�͂��܂��D" << std::endl;
		outputDefaultSettingFile();
		return;
	}


	std::cout << std::endl;
	std::cout << "�ݒ�t�@�C���̓ǂݍ��݂��������܂����D" << std::endl;
}



void ApplicationSettingReader::outputDefaultSettingFile()
{
	const SApplicationSettingRecorder kDefaultSetting;

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
			{ ApplicationSettingKey::FILE_TITLE.key, ApplicationSettingKey::FILE_TITLE_VALUE }
		};
		title_data.comments().push_back(ApplicationSettingKey::FILE_TITLE.description);

		res_str += toml::format(title_data, 0);
		res_str += u8"\n";
	}


	//�o�[�W����
	{
		toml::basic_value<toml::preserve_comments> version_data{
			{
				ApplicationSettingKey::VERSION_TABLE.table_name,
				{
					{ ApplicationSettingKey::VERSION_MAJOR.key, kDefaultSetting.version_major},
					{ ApplicationSettingKey::VERSION_MINOR.key, kDefaultSetting.version_minor },
					{ ApplicationSettingKey::VERSION_PATCH.key, kDefaultSetting.version_patch }
				}
			}
		};

		version_data.at(ApplicationSettingKey::VERSION_TABLE.table_name).comments().push_back(ApplicationSettingKey::VERSION_TABLE.description);
		version_data.at(ApplicationSettingKey::VERSION_TABLE.table_name).at(ApplicationSettingKey::VERSION_MAJOR.key).comments().push_back(ApplicationSettingKey::VERSION_MAJOR.description);
		version_data.at(ApplicationSettingKey::VERSION_TABLE.table_name).at(ApplicationSettingKey::VERSION_MINOR.key).comments().push_back(ApplicationSettingKey::VERSION_MINOR.description);
		version_data.at(ApplicationSettingKey::VERSION_TABLE.table_name).at(ApplicationSettingKey::VERSION_PATCH.key).comments().push_back(ApplicationSettingKey::VERSION_PATCH.description);

		res_str += toml::format(version_data, 0);
	}

	//���[�h
	{
		toml::basic_value<toml::preserve_comments> mode_data{
			{
				ApplicationSettingKey::MODE_TABLE.table_name,
				{
					{ ApplicationSettingKey::ASK_ABOUT_MODES.key, kDefaultSetting.ask_about_modes },
					{ ApplicationSettingKey::DEFAULT_MODE.key, std::to_string(kDefaultSetting.default_mode) },
					{ ApplicationSettingKey::DO_STEP_EXECUTION.key, kDefaultSetting.do_step_execution },
					{ApplicationSettingKey::DO_STEP_EXECUTION_EACH_GAIT.key, kDefaultSetting.do_step_execution_each_gait}
				}
			}
		};

		mode_data.at(ApplicationSettingKey::MODE_TABLE.table_name).comments().push_back(ApplicationSettingKey::MODE_TABLE.description);
		mode_data.at(ApplicationSettingKey::MODE_TABLE.table_name).at(ApplicationSettingKey::ASK_ABOUT_MODES.key).comments().push_back(ApplicationSettingKey::ASK_ABOUT_MODES.description);
		mode_data.at(ApplicationSettingKey::MODE_TABLE.table_name).at(ApplicationSettingKey::DEFAULT_MODE.key).comments().push_back(ApplicationSettingKey::DEFAULT_MODE.description);
		mode_data.at(ApplicationSettingKey::MODE_TABLE.table_name).at(ApplicationSettingKey::DO_STEP_EXECUTION.key).comments().push_back(ApplicationSettingKey::DO_STEP_EXECUTION.description);
		mode_data.at(ApplicationSettingKey::MODE_TABLE.table_name).at(ApplicationSettingKey::DO_STEP_EXECUTION_EACH_GAIT.key).comments().push_back(ApplicationSettingKey::DO_STEP_EXECUTION_EACH_GAIT.description);

		res_str += toml::format(mode_data, 0);
	}

	// �\��
	{
		toml::basic_value<toml::preserve_comments> display_data{
			{
				ApplicationSettingKey::DISPLAY_TABLE.table_name,
				{
					{ ApplicationSettingKey::CMD_OUTPUT.key, kDefaultSetting.cmd_output},
					{ ApplicationSettingKey::CMD_PERMISSION.key, std::to_string(kDefaultSetting.cmd_permission) },
					{ ApplicationSettingKey::GUI_DISPLAY.key, kDefaultSetting.gui_display },
					{ ApplicationSettingKey::GUI_DISPLAY_QUALITY.key, kDefaultSetting.gui_display_quality },
					{ ApplicationSettingKey::WINDOW_SIZE_X.key,kDefaultSetting.window_size_x },
					{ ApplicationSettingKey::WINDOW_SIZE_Y.key,kDefaultSetting.window_size_y },
					{ ApplicationSettingKey::WINDOW_FPS.key,kDefaultSetting.window_fps }
				}
			}
		};

		display_data.at(ApplicationSettingKey::DISPLAY_TABLE.table_name).comments().push_back(ApplicationSettingKey::DISPLAY_TABLE.description);
		display_data.at(ApplicationSettingKey::DISPLAY_TABLE.table_name).at(ApplicationSettingKey::CMD_OUTPUT.key).comments().push_back(ApplicationSettingKey::CMD_OUTPUT.description);
		display_data.at(ApplicationSettingKey::DISPLAY_TABLE.table_name).at(ApplicationSettingKey::CMD_PERMISSION.key).comments().push_back(ApplicationSettingKey::CMD_PERMISSION.description);
		display_data.at(ApplicationSettingKey::DISPLAY_TABLE.table_name).at(ApplicationSettingKey::GUI_DISPLAY.key).comments().push_back(ApplicationSettingKey::GUI_DISPLAY.description);
		display_data.at(ApplicationSettingKey::DISPLAY_TABLE.table_name).at(ApplicationSettingKey::GUI_DISPLAY_QUALITY.key).comments().push_back(ApplicationSettingKey::GUI_DISPLAY_QUALITY.description);
		display_data.at(ApplicationSettingKey::DISPLAY_TABLE.table_name).at(ApplicationSettingKey::WINDOW_SIZE_X.key).comments().push_back(ApplicationSettingKey::WINDOW_SIZE_X.description);
		display_data.at(ApplicationSettingKey::DISPLAY_TABLE.table_name).at(ApplicationSettingKey::WINDOW_SIZE_Y.key).comments().push_back(ApplicationSettingKey::WINDOW_SIZE_Y.description);
		display_data.at(ApplicationSettingKey::DISPLAY_TABLE.table_name).at(ApplicationSettingKey::WINDOW_FPS.key).comments().push_back(ApplicationSettingKey::WINDOW_FPS.description);

		res_str += toml::format(display_data, 0);
	}

	std::ofstream ofs;
	ofs.open(SETTING_FILE_NAME);

	// �t�@�C�����J���Ȃ������牽�����Ȃ�
	if (!ofs)
	{
		std::cout << "�f�t�H���g�̐ݒ�t�@�C���̏o�͂Ɏ��s���܂����D" << std::endl;
		return;
	}

	ofs.write(res_str.c_str(), res_str.length());	// �t�@�C���ɏ�������

	ofs.close();	// �t�@�C�������

	return;
}


void ApplicationSettingReader::readVersionSetting(const toml::value& value, SApplicationSettingRecorder* recorder)
{
	std::cout << std::endl;

	if (value.contains(ApplicationSettingKey::VERSION_TABLE.table_name))
	{
		std::cout << "�Z�o�[�W�����ݒ��ǂݍ��݂܂��D" << std::endl;

		// �o�[�W�����ݒ��ǂݍ���
		if (value.at(ApplicationSettingKey::VERSION_TABLE.table_name).contains(ApplicationSettingKey::VERSION_MAJOR.key))
		{
			recorder->version_major = (int)value.at(ApplicationSettingKey::VERSION_TABLE.table_name).at(ApplicationSettingKey::VERSION_MAJOR.key).as_integer();
			std::cout << "�Z���W���[�o�[�W������ǂݍ��݂܂����Dvalue = " << recorder->version_major << std::endl;
		}
		else
		{
			std::cout << "�~���W���[�o�[�W������������܂���ł����D" << std::endl;
		}

		if (value.at(ApplicationSettingKey::VERSION_TABLE.table_name).contains(ApplicationSettingKey::VERSION_MINOR.key))
		{
			recorder->version_minor = (int)value.at(ApplicationSettingKey::VERSION_TABLE.table_name).at(ApplicationSettingKey::VERSION_MINOR.key).as_integer();
			std::cout << "�Z�}�C�i�[�o�[�W������ǂݍ��݂܂����Dvalue = " << recorder->version_minor << std::endl;
		}
		else
		{
			std::cout << "�~�}�C�i�[�o�[�W������������܂���ł����D" << std::endl;
		}

		if (value.at(ApplicationSettingKey::VERSION_TABLE.table_name).contains(ApplicationSettingKey::VERSION_PATCH.key))
		{
			recorder->version_patch = (int)value.at(ApplicationSettingKey::VERSION_TABLE.table_name).at(ApplicationSettingKey::VERSION_PATCH.key).as_integer();
			std::cout << "�Z�p�b�`�o�[�W������ǂݍ��݂܂����Dvalue = " << recorder->version_patch << std::endl;
		}
		else
		{
			std::cout << "�~�p�b�`�o�[�W������������܂���ł����D" << std::endl;
		}
	}
	else
	{
		std::cout << "�~�o�[�W�����ݒ肪������܂���ł����D" << std::endl;
	}
}


void ApplicationSettingReader::readBootModeSetting(const toml::value& value, SApplicationSettingRecorder* recorder)
{
	std::cout << std::endl;

	if (value.contains(ApplicationSettingKey::MODE_TABLE.table_name))
	{
		std::cout << "�Z�N�����[�h�ݒ��ǂݍ��݂܂��D" << std::endl;

		if (value.at(ApplicationSettingKey::MODE_TABLE.table_name).contains(ApplicationSettingKey::ASK_ABOUT_MODES.key))
		{
			recorder->ask_about_modes = value.at(ApplicationSettingKey::MODE_TABLE.table_name).at(ApplicationSettingKey::ASK_ABOUT_MODES.key).as_boolean();
			std::cout << "�Z�N�����[�h�I���̊m�F�t���O��ǂݍ��݂܂����Dvalue = " << recorder->ask_about_modes << std::endl;
		}
		else
		{
			std::cout << "�~�N�����[�h�I���̊m�F�t���O��������܂���ł����D" << std::endl;
		}

		if (value.at(ApplicationSettingKey::MODE_TABLE.table_name).contains(ApplicationSettingKey::DEFAULT_MODE.key))
		{
			recorder->default_mode = std::sToMode(value.at(ApplicationSettingKey::MODE_TABLE.table_name).at(ApplicationSettingKey::DEFAULT_MODE.key).as_string());
			std::cout << "�Z�f�t�H���g�̋N�����[�h��ǂݍ��݂܂����Dvalue = " << std::to_string(recorder->default_mode) << std::endl;
		}
		else
		{
			std::cout << "�~�f�t�H���g�̋N�����[�h��������܂���ł����D" << std::endl;
		}

		if (value.at(ApplicationSettingKey::MODE_TABLE.table_name).contains(ApplicationSettingKey::DO_STEP_EXECUTION.key))
		{
			recorder->do_step_execution = value.at(ApplicationSettingKey::MODE_TABLE.table_name).at(ApplicationSettingKey::DO_STEP_EXECUTION.key).as_boolean();
			std::cout << "�Z�X�e�b�v���s�t���O��ǂݍ��݂܂����Dvalue = " << std::boolalpha << recorder->do_step_execution << std::endl;
		}
		else
		{
			std::cout << "�~�X�e�b�v���s�t���O��������܂���ł����D" << std::endl;
		}

		if (value.at(ApplicationSettingKey::MODE_TABLE.table_name).contains(ApplicationSettingKey::DO_STEP_EXECUTION_EACH_GAIT.key))
		{
			recorder->do_step_execution_each_gait = value.at(ApplicationSettingKey::MODE_TABLE.table_name).at(ApplicationSettingKey::DO_STEP_EXECUTION_EACH_GAIT.key).as_boolean();
			std::cout << "�Z�X�e�b�v���s�t���O(�e���e)��ǂݍ��݂܂����Dvalue = " << std::boolalpha << recorder->do_step_execution_each_gait << std::endl;
		}
		else
		{
			std::cout << "�~�X�e�b�v���s�t���O(�e���e)��������܂���ł����D" << std::endl;
		}

	}
	else
	{
		std::cout << "�~�N�����[�h�ݒ肪������܂���ł����D" << std::endl;
	}
}


void ApplicationSettingReader::readDisplaySetting(const toml::value& value, SApplicationSettingRecorder* recorder)
{
	std::cout << std::endl;

	if (value.contains(ApplicationSettingKey::DISPLAY_TABLE.table_name))
	{
		std::cout << "�Z�\���ݒ��ǂݍ��݂܂��D" << std::endl;

		if (value.at(ApplicationSettingKey::DISPLAY_TABLE.table_name).contains(ApplicationSettingKey::CMD_OUTPUT.key))
		{
			recorder->cmd_output = value.at(ApplicationSettingKey::DISPLAY_TABLE.table_name).at(ApplicationSettingKey::CMD_OUTPUT.key).as_boolean();
			std::cout << "�Z�R�}���h�o�͂̃t���O��ǂݍ��݂܂����Dvalue = " << std::boolalpha << recorder->cmd_output << std::endl;
		}
		else
		{
			std::cout << "�~�R�}���h�o�͂̃t���O��������܂���ł����D" << std::endl;
		}

		if (value.at(ApplicationSettingKey::DISPLAY_TABLE.table_name).contains(ApplicationSettingKey::CMD_PERMISSION.key))
		{
			recorder->cmd_permission = std::toOutputPriority(value.at(ApplicationSettingKey::DISPLAY_TABLE.table_name).at(ApplicationSettingKey::CMD_PERMISSION.key).as_string());
			std::cout << "�Z�R�}���h�\�������̏���ǂݍ��݂܂����Dvalue = " << std::to_string(recorder->cmd_permission) << std::endl;
		}
		else
		{
			std::cout << "�~�R�}���h�\�������̏�񂪌�����܂���ł����D" << std::endl;
		}

		if (value.at(ApplicationSettingKey::DISPLAY_TABLE.table_name).contains(ApplicationSettingKey::GUI_DISPLAY.key))
		{
			recorder->gui_display = value.at(ApplicationSettingKey::DISPLAY_TABLE.table_name).at(ApplicationSettingKey::GUI_DISPLAY.key).as_boolean();
			std::cout << "�ZGUI�\���̃t���O��ǂݍ��݂܂����Dvalue = " << std::boolalpha << recorder->gui_display << std::endl;
		}
		else
		{
			std::cout << "�~GUI�\���̃t���O��������܂���ł����D" << std::endl;
		}

		if (value.at(ApplicationSettingKey::GUI_DISPLAY_QUALITY.table_name).contains(ApplicationSettingKey::GUI_DISPLAY_QUALITY.key))
		{
			recorder->gui_display_quality = value.at(ApplicationSettingKey::GUI_DISPLAY_QUALITY.table_name).at(ApplicationSettingKey::GUI_DISPLAY_QUALITY.key).as_string();
			std::cout << "�ZGUI�\���i����ǂݍ��݂܂����Dvalue = " << recorder->gui_display_quality << std::endl;
		}
		else
		{
			std::cout << "�~GUI�\���i����������܂���ł����D" << std::endl;
		}

		if (value.at(ApplicationSettingKey::DISPLAY_TABLE.table_name).contains(ApplicationSettingKey::WINDOW_SIZE_X.key))
		{
			recorder->window_size_x = (int)value.at(ApplicationSettingKey::DISPLAY_TABLE.table_name).at(ApplicationSettingKey::WINDOW_SIZE_X.key).as_integer();
			std::cout << "�Z�E�B���h�E��X�T�C�Y(����)�̒l��ǂݍ��݂܂����Dvalue = " << recorder->window_size_x << std::endl;
		}
		else
		{
			std::cout << "�~�E�B���h�E��X�T�C�Y(����)�̒l��������܂���ł����D" << std::endl;
		}

		if (value.at(ApplicationSettingKey::DISPLAY_TABLE.table_name).contains(ApplicationSettingKey::WINDOW_SIZE_Y.key))
		{
			recorder->window_size_y = (int)value.at(ApplicationSettingKey::DISPLAY_TABLE.table_name).at(ApplicationSettingKey::WINDOW_SIZE_Y.key).as_integer();
			std::cout << "�Z�E�B���h�E��Y�T�C�Y(�c��)�̒l��ǂݍ��݂܂����Dvalue = " << recorder->window_size_y << std::endl;
		}
		else
		{
			std::cout << "�~�E�B���h�E��Y�T�C�Y(�c��)�̒l��������܂���ł����D" << std::endl;
		}

		if (value.at(ApplicationSettingKey::DISPLAY_TABLE.table_name).contains(ApplicationSettingKey::WINDOW_FPS.key))
		{
			recorder->window_fps = (int)value.at(ApplicationSettingKey::DISPLAY_TABLE.table_name).at(ApplicationSettingKey::WINDOW_FPS.key).as_integer();
			std::cout << "�Z�E�B���h�E��FPS�̒l��ǂݍ��݂܂����Dvalue = " << recorder->window_fps << std::endl;
		}
		else
		{
			std::cout << "�~�E�B���h�E��FPS�̒l��������܂���ł����D" << std::endl;
		}
	}
	else
	{
		std::cout << "�~�\���ݒ肪������܂���ł����D" << std::endl;
	}
}
