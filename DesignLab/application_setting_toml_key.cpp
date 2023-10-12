#include "application_setting_toml_key.h"


const std::string ApplicationSettingTomlKey::kFileTitleValue = "graph_search_settings";

const TomlSettingKeyData ApplicationSettingTomlKey::kFileTitle = { "" ,"title", u8"�^�C�g����\"" + ApplicationSettingTomlKey::kFileTitleValue + u8"\"�̕��݂̂�ǂݍ��݂܂�" };


const TomlSettingTableData ApplicationSettingTomlKey::kVersionTable = { "version" ,u8"ver major.minor.patch �̏��ŋL�q����܂��D���Ɏg���\��͂Ȃ��f�[�^������ǈꉞ�p�ӂ��Ă���" };

const TomlSettingTableData ApplicationSettingTomlKey::kMoveTable = { "mode" ,u8"�v���O�����̋N�����[�h�̐ݒ�ł�" };

const TomlSettingTableData ApplicationSettingTomlKey::kDisplayTable = { "display" ,u8"�\���Ɋւ���ݒ�ł��D�O���t�T��������ۂ̐ݒ�ŁCviewer���[�h�Ŏ��s�����ꍇ�͖�������܂��D" };


const TomlSettingKeyData ApplicationSettingTomlKey::kVersionMajor = { ApplicationSettingTomlKey::kVersionTable.table_name ,"major",u8"" };

const TomlSettingKeyData ApplicationSettingTomlKey::kVersionMinor = { ApplicationSettingTomlKey::kVersionTable.table_name ,"minor",u8"" };

const TomlSettingKeyData ApplicationSettingTomlKey::kVersionPatch = { ApplicationSettingTomlKey::kVersionTable.table_name ,"patch",u8"" };


const TomlSettingKeyData ApplicationSettingTomlKey::kAskAboutBootMode = { ApplicationSettingTomlKey::kMoveTable.table_name ,"ask_about_modes",u8"�N�����Ɏ��s���[�h�ɂ��Ď��������悤�ɂ��܂� (true/false)" };

const TomlSettingKeyData ApplicationSettingTomlKey::kDefaultMode = { ApplicationSettingTomlKey::kMoveTable.table_name ,"default_mode",u8"�f�t�H���g�̎��s���[�h��ݒ肵�܂��D(simulation/viewer/display_test/result_viewer)" };

const TomlSettingKeyData ApplicationSettingTomlKey::kDoStepExecution = { ApplicationSettingTomlKey::kMoveTable.table_name ,"do_step_execution",u8"�V�~�����[�V�������X�e�b�v���s���邩�ǂ�����ݒ肵�܂� (true/false)" };

const TomlSettingKeyData ApplicationSettingTomlKey::kDoStepEexcutionEachGait = { ApplicationSettingTomlKey::kMoveTable.table_name ,"do_step_execution_each_gait",u8"�e���e�������ƂɃX�e�b�v���s���邩�ǂ�����ݒ肵�܂� (true/false)" };


const TomlSettingKeyData ApplicationSettingTomlKey::kOutputCmd = { ApplicationSettingTomlKey::kDisplayTable.table_name ,"cmd_output",u8"�R�}���h���C���ւ̏o�͂��s���܂� (true/false)" };

const TomlSettingKeyData ApplicationSettingTomlKey::kCmdPermission = { ApplicationSettingTomlKey::kDisplayTable.table_name ,"cmd_permission",u8"�R�}���h���C���ɏo�͂��镶������ǂ��܂ŋ����邩�C(debug,info,warning,error,system)�̏��ɗD�揇�ʂ�����" };

const TomlSettingKeyData ApplicationSettingTomlKey::kDisplayGui = { ApplicationSettingTomlKey::kDisplayTable.table_name ,"gui_display",u8"GUI(dxlib�ɂ��\��)�ł̕\�����s���܂� (true/false)" };

const TomlSettingKeyData ApplicationSettingTomlKey::kGuiDisplayQuality = { ApplicationSettingTomlKey::kDisplayTable.table_name ,"gui_display_quality",u8"GUI�ł̕\���̕i����ݒ肵�܂��D(low,medium,high)�̏��ɕi���������Ȃ�܂��D" };

const TomlSettingKeyData ApplicationSettingTomlKey::kWindowSizeX = { ApplicationSettingTomlKey::kDisplayTable.table_name ,"window_size_x",u8"GUI�̃E�B���h�E�̃T�C�Y�̉�����ݒ肵�܂��Dx��y��16�F9 �ɂȂ�悤�ɐݒ肵�Ă��������D" };

const TomlSettingKeyData ApplicationSettingTomlKey::kWindowSizeY = { ApplicationSettingTomlKey::kDisplayTable.table_name ,"window_size_y",u8"GUI�̃E�B���h�E�̃T�C�Y�̏c����ݒ肵�܂��D�����l�� (x, y) = (960, 540) (1280, 720) (1600, 900) (1920, 1080) �Ȃǂł��D" };

const TomlSettingKeyData ApplicationSettingTomlKey::kWindowFps = { ApplicationSettingTomlKey::kDisplayTable.table_name ,"window_pos_x",u8"�E�B���h�E�̃t���[�����[�g��ݒ肵�܂��D�����l��60��30�ł��D" };