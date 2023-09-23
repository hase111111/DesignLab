//! @file application_setting_key.h
//! @brief �ݒ�t�@�C���̃L�[���܂Ƃ߂��萔�N���X

#ifndef DESIGNLAB_APPLICATION_SETTING_KEY_H_
#define DESIGNLAB_APPLICATION_SETTING_KEY_H_


#include <string>


//! @struct SettingKeyData
//! @brief �ݒ�t�@�C���̃f�[�^�̃L�[���܂Ƃ߂��\����
struct SettingKeyData
{
	std::string table_name;		//!< �f�[�^�������Ă���e�[�u����
	std::string key;			//!< �L�[��
	std::string description;	//!< ����
};


//! @struct SettingTableData
//! @brief �ݒ�t�@�C���̃e�[�u���̃f�[�^���܂Ƃ߂��\����
struct SettingTableData
{
	std::string table_name;		//!< �e�[�u����
	std::string description;	//!< ����
};


//! @class ApplicationSettingKey
//! @brief �ݒ�t�@�C���̃L�[���܂Ƃ߂��萔�N���X
class ApplicationSettingKey final
{
public:

	//�R���X�g���N�^�͍폜���C���̂𐶐����Ȃ�
	ApplicationSettingKey() = delete;
	ApplicationSettingKey(ApplicationSettingKey& other) = delete;
	ApplicationSettingKey(ApplicationSettingKey&& other) = delete;
	ApplicationSettingKey& operator=(ApplicationSettingKey& other) = delete;

	const static std::string FILE_TITLE_VALUE;
	const static SettingKeyData FILE_TITLE;

	const static SettingTableData VERSION_TABLE;
	const static SettingKeyData VERSION_MAJOR;
	const static SettingKeyData VERSION_MINOR;
	const static SettingKeyData VERSION_PATCH;

	const static SettingTableData MODE_TABLE;
	const static SettingKeyData ASK_ABOUT_MODES;
	const static SettingKeyData DEFAULT_MODE;
	const static SettingKeyData DO_STEP_EXECUTION;
	const static SettingKeyData DO_STEP_EXECUTION_EACH_GAIT;

	const static SettingTableData DISPLAY_TABLE;
	const static SettingKeyData CMD_OUTPUT;
	const static SettingKeyData CMD_PERMISSION;
	const static SettingKeyData GUI_DISPLAY;
	const static SettingKeyData GUI_DISPLAY_QUALITY;
	const static SettingKeyData WINDOW_SIZE_X;
	const static SettingKeyData WINDOW_SIZE_Y;
	const static SettingKeyData WINDOW_FPS;
};


#endif // !DESIGNLAB_APPLICATION_SETTING_KEY_H_