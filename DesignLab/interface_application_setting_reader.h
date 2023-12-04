//! @file interface_application_setting_reader.h
//! @brief �ݒ�t�@�C����ǂݍ��ރN���X�̃C���^�[�t�F�[�X�D


#ifndef DESIGNLAB_INTERFACE_APPLICATION_SETTING_READER_H_
#define DESIGNLAB_INTERFACE_APPLICATION_SETTING_READER_H_


#include <memory>

#include "application_setting_recorder.h"


//! @class IApplicationSettingReader
//! @brief �ݒ�t�@�C����ǂݍ��ރN���X�̃C���^�[�t�F�[�X�D
//! @n ���ݎv���t����toml�t�@�C���œǂݍ��݂����Ă��邪�C��X�ύX����\��������̂ŁC���̃C���^�[�t�F�[�X��p�ӂ��Ă���D
//! @n json�Ƃ��ɕς����炱�̃N���X���p�����āCjson�t�@�C����ǂݍ��ރN���X���쐬���ĂˁD
class IApplicationSettingReader
{
public:

	virtual ~IApplicationSettingReader() = default;

	//! @brief �ݒ�t�@�C����ǂݍ��ށD
	//! @n �ݒ�t�@�C�������݂��Ȃ��ꍇ�̓f�t�H���g�̐ݒ�t�@�C�����o�͂���D
	//! @return std::shared_ptr<ApplicationSettingRecorder> �ݒ�t�@�C���̓��e�D
	virtual std::shared_ptr<ApplicationSettingRecorder> ReadFileOrUseAndOutputDefault() = 0;
};


#endif	// DESIGNLAB_INTERFACE_APPLICATION_SETTING_READER_H_