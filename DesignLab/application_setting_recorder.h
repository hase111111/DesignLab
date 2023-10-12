//! @file application_setting_recorder.h
//! @brief �A�v���̐ݒ���L�^����\����


#ifndef DESIGNLAB_APPLICATION_SETTING_RECORDER_H_
#define DESIGNLAB_APPLICATION_SETTING_RECORDER_H_


#include <string>

#include "application_setting_toml_key.h"
#include "boot_mode.h"
#include "output_detail.h"


//! @struct ApplicationSettingRecorder
//! @brief �A�v���̐ݒ���L�^����\����
struct ApplicationSettingRecorder
{
	const std::string kSettingFileTitle = ApplicationSettingTomlKey::kFileTitleValue;		//!< �ݒ�t�@�C���̃^�C�g��

	int version_major = 0;				//!< �o�[�W�����ԍ�(���W���[)
	int version_minor = 5;				//!< �o�[�W�����ԍ�(�}�C�i�[)
	int version_patch = 0;				//!< �o�[�W�����ԍ�(�p�b�`)


	bool ask_about_modes = true;					//!< �N�����Ƀ��[�h�I���̊m�F�����邩�ǂ���
	BootMode default_mode = BootMode::kSimulation;	//!< �f�t�H���g�̋N�����[�h
	bool do_step_execution = true;					//!< 1�V�~�����[�V�������ƂɃX�e�b�v���s�����邩�ǂ���
	bool do_step_execution_each_gait = false;		//!< 1���삲�ƂɃX�e�b�v���s�����邩�ǂ���


	bool cmd_output = true;								//!< �R�}���h���C���ɏo�͂��邩�ǂ���
	OutputDetail cmd_permission = OutputDetail::kDebug;	//!< �R�}���h���C���ɏo�͂���ہC�ǂ��܂ŋ����邩
	bool gui_display = true;							//!< GUI��\�����邩�ǂ���
	std::string gui_display_quality = "high";			//!< GUI��\������ہC�ǂ��܂ŋ����邩
	int window_size_x = 1280;							//!< �O���t�B�J���E�B���h�E�̉���
	int window_size_y = 720;							//!< �O���t�B�J���E�B���h�E�̏c��
	int window_fps = 60; 								//!< �O���t�B�J���E�B���h�E��FPS	
};


#endif