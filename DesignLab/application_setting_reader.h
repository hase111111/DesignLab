#pragma once

#include <string>

#include "application_setting_recorder.h"
#include "application_setting_key.h"

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

	//���s����
	void outputIndention(std::ofstream& ofs, int indention) const;

	//�e�[�u�����o�͂���
	void outputTable(std::ofstream& ofs, const SettingTableData& table) const;


	const std::string SETTING_FILE_NAME = "settings.toml";


};



//! @file application_setting_reader.h
//! @date 2023/08/25
//! @author ���J��
//! @brief �A�v���P�[�V�����ݒ�t�@�C����ǂݍ��ރN���X
//! @n �s�� : @lineinfo