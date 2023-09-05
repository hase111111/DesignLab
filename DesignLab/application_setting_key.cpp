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

const SettingKeyData ApplicationSettingKey::CMD_PERMISSION = { ApplicationSettingKey::DISPLAY_TABLE.table_name ,"cmd_permission",u8"�R�}���h���C���ɏo�͂��镶������ǂ��܂ŋ����邩�C(debug,info,error,warning,system)�̏��ɗD�揇�ʂ�����" };

const SettingKeyData ApplicationSettingKey::GUI_DISPLAY = { ApplicationSettingKey::DISPLAY_TABLE.table_name ,"gui_display",u8"GUI(dxlib�ɂ��\��)�ł̕\�����s���܂� (true/false)" };

const SettingKeyData ApplicationSettingKey::GUI_DISPLAY_QUALITY = { ApplicationSettingKey::DISPLAY_TABLE.table_name ,"gui_display_quality",u8"GUI�ł̕\���̕i����ݒ肵�܂��D(low,medium,high)�̏��ɕi���������Ȃ�܂��D" };

const SettingKeyData ApplicationSettingKey::WINDOW_SIZE_X = { ApplicationSettingKey::DISPLAY_TABLE.table_name ,"window_size_x",u8"GUI�̃E�B���h�E�̃T�C�Y�̉�����ݒ肵�܂��Dx��y��16�F9 �ɂȂ�悤�ɐݒ肵�Ă��������D" };

const SettingKeyData ApplicationSettingKey::WINDOW_SIZE_Y = { ApplicationSettingKey::DISPLAY_TABLE.table_name ,"window_size_y",u8"GUI�̃E�B���h�E�̃T�C�Y�̏c����ݒ肵�܂��D�����l�� (x, y) = (960, 540) (1280, 720) (1600, 900) (1920, 1080) �Ȃǂł��D" };

const SettingKeyData ApplicationSettingKey::WINDOW_FPS = { ApplicationSettingKey::DISPLAY_TABLE.table_name ,"window_pos_x",u8"�E�B���h�E�̃t���[�����[�g��ݒ肵�܂��D�����l��60��30�ł��D" };


const SettingTableData ApplicationSettingKey::MAP_TABLE = { "map" ,"�}�b�v�����Ɋւ���ݒ�ł��D�O���t�T��������ۂ̐ݒ�ŁCviewer���[�h�Ŏ��s�����ꍇ�͖�������܂��D" };

const SettingKeyData ApplicationSettingKey::MAP_CREATE_MODE = { ApplicationSettingKey::MAP_TABLE.table_name ,"map_create_mode","�}�b�v�̐������@��ݒ肵�܂��D�����Ŏw�肵�܂�" };

const SettingKeyData ApplicationSettingKey::MAP_CREATE_OPTION = { ApplicationSettingKey::MAP_TABLE.table_name ,"map_create_option","�}�b�v�̐������@�̃I�v�V������ݒ肵�܂��D�����Ŏw�肵�܂�" };

const SettingKeyData ApplicationSettingKey::DO_OUTPUT_MAP = { ApplicationSettingKey::MAP_TABLE.table_name ,"do_output_map","�}�b�v�̐������Ƀ}�b�v���o�͂��邩�ǂ�����ݒ肵�܂��D" };

const SettingKeyData ApplicationSettingKey::MAP_HOLE_RATE = { ApplicationSettingKey::MAP_TABLE.table_name ,"map_hole_rate","�}�b�v�̐������Ɍ��𐶐�����m����ݒ肵�܂��D0 ~ 100�͈̔͂Ŏw�肵�܂��D" };

const SettingKeyData ApplicationSettingKey::MAP_STEP_HEIGHT = { ApplicationSettingKey::MAP_TABLE.table_name ,"map_step_height","�}�b�v�̐������ɒi���𐶐����鍂����ݒ肵�܂��D�����Ŏw�肵�܂�" };

const SettingKeyData ApplicationSettingKey::MAP_STEP_LENGTH = { ApplicationSettingKey::MAP_TABLE.table_name ,"map_step_length","�}�b�v�̐������ɒi���𐶐����钷����ݒ肵�܂��D�����Ŏw�肵�܂�" };

const SettingKeyData ApplicationSettingKey::MAP_SLOPE_ANGLE_DEG = { ApplicationSettingKey::MAP_TABLE.table_name ,"map_slope_angle_deg","�}�b�v�̐������ɎΖʂ𐶐�����p�x[deg]��ݒ肵�܂��D�����Ŏw�肵�܂�" };

const SettingKeyData ApplicationSettingKey::MAP_TILT_ANGLE_DEG = { ApplicationSettingKey::MAP_TABLE.table_name ,"map_tilt_angle_deg","�}�b�v�̐������ɌX�΂𐶐�����p�x[deg]��ݒ肵�܂��D�����Ŏw�肵�܂�" };

const SettingKeyData ApplicationSettingKey::ROUGH_MAX_DIF = { ApplicationSettingKey::MAP_TABLE.table_name ,"rough_max_dif","�}�b�v�̐������ɒi���𐶐�����ۂ̍ő�̍����̍���ݒ肵�܂��D�����Ŏw�肵�܂�" };

const SettingKeyData ApplicationSettingKey::ROUGH_MIN_DIF = { ApplicationSettingKey::MAP_TABLE.table_name ,"rough_min_dif","�}�b�v�̐������ɒi���𐶐�����ۂ̍ŏ��̍����̍���ݒ肵�܂��D�����Ŏw�肵�܂�" };

