//! @file application_setting_reader.h
//! @brief �A�v���P�[�V�����ݒ�t�@�C����ǂݍ��ރN���X

#ifndef APPLICATION_SETTING_READER_H_
#define APPLICATION_SETTING_READER_H_


#include <memory>
#include <string>

#include "toml.hpp"

#include "application_setting_recorder.h"
#include "application_setting_key.h"


// toml11�ŃR�����g��ێ����邽�߂̃}�N��
#define TOML11_PRESERVE_COMMENTS_BY_DEFAULT


//! @class ApplicationSettingReader
//! @brief �A�v���P�[�V�����ݒ�t�@�C����ǂݍ��ރN���X
class ApplicationSettingReader final
{
public:

	//! @brief �ݒ�t�@�C����ǂݍ���
	//! @n �ݒ�t�@�C�������݂��Ȃ��ꍇ�̓f�t�H���g�̐ݒ�t�@�C�����o�͂���
	//! @return std::shared_ptr<ApplicationSettingRecorder> �ݒ�t�@�C���̓��e
	std::shared_ptr<ApplicationSettingRecorder> Read();

private:

	//�t�@�C�������݂��Ȃ������ꍇ�̂��߂Ƀf�t�H���g�̐ݒ�t�@�C�����o�͂���
	void OutputDefaultSettingFile();

	//�ݒ�t�@�C������o�[�W��������ǂݍ���
	void ReadVersionSetting(const toml::value& value, std::shared_ptr<ApplicationSettingRecorder>& recorder);

	//�ݒ�t�@�C������N�����[�h�̏���ǂݍ���
	void ReadBootModeSetting(const toml::value& value, std::shared_ptr<ApplicationSettingRecorder>& recorder);

	//�ݒ�t�@�C������f�B�X�v���C����ǂݍ���
	void ReadDisplaySetting(const toml::value& value, std::shared_ptr<ApplicationSettingRecorder>& recorder);

	const std::string kSettingFileNname = u8"settings.toml";
};


#endif //APPLICATION_SETTING_READER_H_