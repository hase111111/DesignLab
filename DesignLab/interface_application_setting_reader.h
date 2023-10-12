//! @file interface_application_setting_reader.h
//! @brief �ݒ�t�@�C����ǂݍ��ރN���X�̃C���^�[�t�F�[�X


#ifndef DESIGNLAB_INTERFACE_APPLICATION_SETTING_READER_H_
#define DESIGNLAB_INTERFACE_APPLICATION_SETTING_READER_H_


#include <memory>

#include "application_setting_recorder.h"


//! @class IApplicationSettingReader
//! @brief �ݒ�t�@�C����ǂݍ��ރN���X�̃C���^�[�t�F�[�X
//! @n ���ݎv���t����toml�t�@�C���œǂݍ��݂����Ă��邪�C��X�ύX����\��������̂ŁC���̃C���^�[�t�F�[�X��p�ӂ��Ă���
//! @n json�Ƃ��ɕς����炱�̃N���X���p�����āCjson�t�@�C����ǂݍ��ރN���X���쐬���Ă�
class IApplicationSettingReader
{
public:

	virtual ~IApplicationSettingReader() = default;

	//! @brief �ݒ�t�@�C����ǂݍ���
	//! @n �ݒ�t�@�C�������݂��Ȃ��ꍇ�̓f�t�H���g�̐ݒ�t�@�C�����o�͂���
	//! @return std::shared_ptr<ApplicationSettingRecorder> �ݒ�t�@�C���̓��e
	virtual std::shared_ptr<ApplicationSettingRecorder> Read() = 0;
};


#endif