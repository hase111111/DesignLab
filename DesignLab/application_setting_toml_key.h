//! @file application_setting_key.h
//! @brief �ݒ�t�@�C��(toml�t�@�C��)�̃L�[���܂Ƃ߂��萔�N���X

#ifndef DESIGNLAB_APPLICATION_SETTING_TOML_KEY_H_
#define DESIGNLAB_APPLICATION_SETTING_TOML_KEY_H_


#include <string>


//! @struct TomlSettingKeyData
//! @brief �ݒ�t�@�C���̃f�[�^�̃L�[���܂Ƃ߂��\����
struct TomlSettingKeyData final
{
	std::string table_name;		//!< �f�[�^�������Ă���e�[�u����
	std::string key;			//!< �L�[��
	std::string description;	//!< ����
};


//! @struct TomlSettingTableData
//! @brief �ݒ�t�@�C���̃e�[�u���̃f�[�^���܂Ƃ߂��\����
struct TomlSettingTableData final
{
	std::string table_name;		//!< �e�[�u����
	std::string description;	//!< ����
};


//! @class ApplicationSettingTomlKey
//! @brief toml�`���̐ݒ�t�@�C���̃L�[���܂Ƃ߂��萔�N���X
class ApplicationSettingTomlKey final
{
public:

	//�R���X�g���N�^�͍폜���C���̂𐶐����Ȃ�
	ApplicationSettingTomlKey() = delete;
	ApplicationSettingTomlKey(ApplicationSettingTomlKey& other) = delete;
	ApplicationSettingTomlKey(ApplicationSettingTomlKey&& other) = delete;
	ApplicationSettingTomlKey& operator=(ApplicationSettingTomlKey& other) = delete;

	const static std::string kFileTitleValue;	//!< toml�t�@�C���̖��O�ł͂Ȃ��CTitle�Ƃ����L�[�Ɋi�[�����l
	const static TomlSettingKeyData kFileTitle;	

	const static TomlSettingTableData kVersionTable;
	const static TomlSettingTableData kMoveTable;
	const static TomlSettingTableData kDisplayTable;

	const static TomlSettingKeyData kVersionMajor;
	const static TomlSettingKeyData kVersionMinor;
	const static TomlSettingKeyData kVersionPatch;

	const static TomlSettingKeyData kAskAboutBootMode;
	const static TomlSettingKeyData kDefaultMode;
	const static TomlSettingKeyData kDoStepExecution;
	const static TomlSettingKeyData kDoStepEexcutionEachGait;

	const static TomlSettingKeyData kOutputCmd;
	const static TomlSettingKeyData kCmdPermission;
	const static TomlSettingKeyData kDisplayGui;
	const static TomlSettingKeyData kGuiDisplayQuality;
	const static TomlSettingKeyData kWindowSizeX;
	const static TomlSettingKeyData kWindowSizeY;
	const static TomlSettingKeyData kWindowFps;
};


#endif // DESIGNLAB_APPLICATION_SETTING_KEY_H_