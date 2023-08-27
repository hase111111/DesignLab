#pragma once

#include <string>

#include "toml.hpp"

#include "application_setting_recorder.h"
#include "application_setting_key.h"


// toml11�ŃR�����g��ێ����邽�߂̃}�N��
#define TOML11_PRESERVE_COMMENTS_BY_DEFAULT


//! @class ApplicationSettingReader
//! @date 2023/08/25
//! @author ���J��
//! @brief �A�v���P�[�V�����ݒ�t�@�C����ǂݍ��ރN���X
class ApplicationSettingReader final
{
public:

	//! @brief �ݒ�t�@�C����ǂݍ���
	//! @n �ݒ�t�@�C�������݂��Ȃ��ꍇ�̓f�t�H���g�̐ݒ�t�@�C�����o�͂���
	//! @param [out] recorder �ǂݍ��񂾐ݒ�t�@�C���̓��e���i�[����
	void read(SApplicationSettingRecorder* recorder);

private:

	//�t�@�C�������݂��Ȃ������ꍇ�̂��߂Ƀf�t�H���g�̐ݒ�t�@�C�����o�͂���
	void outputDefaultSettingFile();

	//�ݒ�t�@�C������o�[�W��������ǂݍ���
	void readVersionSetting(const toml::value& value, SApplicationSettingRecorder* recorder);

	//�ݒ�t�@�C������N�����[�h�̏���ǂݍ���
	void readBootModeSetting(const toml::value& value, SApplicationSettingRecorder* recorder);

	//�ݒ�t�@�C������f�B�X�v���C����ǂݍ���
	void readDisplaySetting(const toml::value& value, SApplicationSettingRecorder* recorder);

	const std::string SETTING_FILE_NAME = u8"settings.toml";


};



//! @file application_setting_reader.h
//! @date 2023/08/25
//! @author ���J��
//! @brief �A�v���P�[�V�����ݒ�t�@�C����ǂݍ��ރN���X
//! @n �s�� : @lineinfo