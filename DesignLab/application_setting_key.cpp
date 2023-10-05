#include "application_setting_key.h"


const std::string ApplicationSettingKey::FILE_TITLE_VALUE = "graph_search_settings";

const SettingKeyData ApplicationSettingKey::FILE_TITLE = { "" ,"title", u8"�^�C�g����\"" + ApplicationSettingKey::FILE_TITLE_VALUE + u8"\"�̕��݂̂�ǂݍ��݂܂�" };


const SettingTableData ApplicationSettingKey::VERSION_TABLE = { "version" ,u8"ver major.minor.patch �̏��ŋL�q����܂��D���Ɏg���\��͂Ȃ��f�[�^������ǈꉞ�p�ӂ��Ă���" };

const SettingKeyData ApplicationSettingKey::VERSION_MAJOR = { ApplicationSettingKey::VERSION_TABLE.table_name ,"major",u8"" };

const SettingKeyData ApplicationSettingKey::VERSION_MINOR = { ApplicationSettingKey::VERSION_TABLE.table_name ,"minor",u8"" };

const SettingKeyData ApplicationSettingKey::VERSION_PATCH = { ApplicationSettingKey::VERSION_TABLE.table_name ,"patch",u8"" };


const SettingTableData ApplicationSettingKey::MODE_TABLE = { "mode" ,u8"�v���O�����̋N�����[�h�̐ݒ�ł�" };

const SettingKeyData ApplicationSettingKey::ASK_ABOUT_MODES = { ApplicationSettingKey::MODE_TABLE.table_name ,"ask_about_modes",u8"�N�����Ɏ��s���[�h�ɂ��Ď��������悤�ɂ��܂� (true/false)" };

const SettingKeyData ApplicationSettingKey::DEFAULT_MODE = { ApplicationSettingKey::MODE_TABLE.table_name ,"default_mode",u8"�f�t�H���g�̎��s���[�h��ݒ肵�܂��D(simulation/viewer/display_test/result_viewer)" };

const SettingKeyData ApplicationSettingKey::DO_STEP_EXECUTION = { ApplicationSettingKey::MODE_TABLE.table_name ,"do_step_execution",u8"�V�~�����[�V�������X�e�b�v���s���邩�ǂ�����ݒ肵�܂� (true/false)" };

const SettingKeyData ApplicationSettingKey::DO_STEP_EXECUTION_EACH_GAIT = { ApplicationSettingKey::MODE_TABLE.table_name ,"do_step_execution_each_gait",u8"�e���e�������ƂɃX�e�b�v���s���邩�ǂ�����ݒ肵�܂� (true/false)" };



const SettingTableData ApplicationSettingKey::DISPLAY_TABLE = { "display" ,u8"�\���Ɋւ���ݒ�ł��D�O���t�T��������ۂ̐ݒ�ŁCviewer���[�h�Ŏ��s�����ꍇ�͖�������܂��D" };

const SettingKeyData ApplicationSettingKey::CMD_OUTPUT = { ApplicationSettingKey::DISPLAY_TABLE.table_name ,"cmd_output",u8"�R�}���h���C���ւ̏o�͂��s���܂� (true/false)" };

const SettingKeyData ApplicationSettingKey::CMD_PERMISSION = { ApplicationSettingKey::DISPLAY_TABLE.table_name ,"cmd_permission",u8"�R�}���h���C���ɏo�͂��镶������ǂ��܂ŋ����邩�C(debug,info,warning,error,system)�̏��ɗD�揇�ʂ�����" };

const SettingKeyData ApplicationSettingKey::GUI_DISPLAY = { ApplicationSettingKey::DISPLAY_TABLE.table_name ,"gui_display",u8"GUI(dxlib�ɂ��\��)�ł̕\�����s���܂� (true/false)" };

const SettingKeyData ApplicationSettingKey::GUI_DISPLAY_QUALITY = { ApplicationSettingKey::DISPLAY_TABLE.table_name ,"gui_display_quality",u8"GUI�ł̕\���̕i����ݒ肵�܂��D(low,medium,high)�̏��ɕi���������Ȃ�܂��D" };

const SettingKeyData ApplicationSettingKey::WINDOW_SIZE_X = { ApplicationSettingKey::DISPLAY_TABLE.table_name ,"window_size_x",u8"GUI�̃E�B���h�E�̃T�C�Y�̉�����ݒ肵�܂��Dx��y��16�F9 �ɂȂ�悤�ɐݒ肵�Ă��������D" };

const SettingKeyData ApplicationSettingKey::WINDOW_SIZE_Y = { ApplicationSettingKey::DISPLAY_TABLE.table_name ,"window_size_y",u8"GUI�̃E�B���h�E�̃T�C�Y�̏c����ݒ肵�܂��D�����l�� (x, y) = (960, 540) (1280, 720) (1600, 900) (1920, 1080) �Ȃǂł��D" };

const SettingKeyData ApplicationSettingKey::WINDOW_FPS = { ApplicationSettingKey::DISPLAY_TABLE.table_name ,"window_pos_x",u8"�E�B���h�E�̃t���[�����[�g��ݒ肵�܂��D�����l��60��30�ł��D" };