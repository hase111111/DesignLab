//! @file application_setting_reader.h
//! @brief �A�v���P�[�V�����ݒ�t�@�C����ǂݍ��ރN���X

#ifndef APPLICATION_SETTING_READER_TOML_H_
#define APPLICATION_SETTING_READER_TOML_H_


#include <memory>
#include <string>

#include "toml.hpp"

#include "application_setting_recorder.h"
#include "application_setting_toml_key.h"
#include "interface_application_setting_reader.h"


// toml11�ŃR�����g��ێ����邽�߂̃}�N��
#define TOML11_PRESERVE_COMMENTS_BY_DEFAULT


//! @class ApplicationSettingReaderToml
//! @brief toml�`���̃A�v���P�[�V�����ݒ�t�@�C����ǂݍ��ރN���X
class ApplicationSettingReaderToml final : public IApplicationSettingReader
{
public:

	std::shared_ptr<ApplicationSettingRecorder> ReadFileOrUseAndOutputDefault() override;

private:

	//�t�@�C�������݂��Ȃ������ꍇ�̂��߂Ƀf�t�H���g�̐ݒ�t�@�C�����o�͂���
	void OutputDefaultSettingFile();

	//�ݒ�t�@�C������o�[�W��������ǂݍ���
	void ReadVersionSetting(const toml::value& value, std::shared_ptr<ApplicationSettingRecorder>& recorder);

	//�ݒ�t�@�C������N�����[�h�̏���ǂݍ���
	void ReadBootModeSetting(const toml::value& value, std::shared_ptr<ApplicationSettingRecorder>& recorder);

	//�ݒ�t�@�C������f�B�X�v���C����ǂݍ���
	void ReadDisplaySetting(const toml::value& value, std::shared_ptr<ApplicationSettingRecorder>& recorder);

	const std::string kSettingFileName = u8"settings.toml";
};


#endif	// APPLICATION_SETTING_READER_TOML_H_