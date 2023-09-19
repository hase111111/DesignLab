#pragma once

#include <string>

#include "output_detail.h"
#include "boot_mode.h"
#include "map_creator.h"
#include "application_setting_key.h"


//! @struct SApplicationSettingRecorder
//! @date 2023/08/25
//! @author ���J��
//! @brief �A�v���̐ݒ���L�^����\����
struct SApplicationSettingRecorder
{
	const std::string SETTING_FILE_TITLE = ApplicationSettingKey::FILE_TITLE_VALUE;		//!< �ݒ�t�@�C���̃^�C�g��

	int version_major = 0;				//!< �o�[�W�����ԍ�(���W���[)
	int version_minor = 5;				//!< �o�[�W�����ԍ�(�}�C�i�[)
	int version_patch = 0;				//!< �o�[�W�����ԍ�(�p�b�`)


	bool ask_about_modes = true;					//!< �N�����Ƀ��[�h�I���̊m�F�����邩�ǂ���
	EBootMode default_mode = EBootMode::SIMULATION;	//!< �f�t�H���g�̋N�����[�h
	bool do_step_execution = true;					//!< 1�V�~�����[�V�������ƂɃX�e�b�v���s�����邩�ǂ���
	bool do_step_execution_each_gait = false;		//!< 1���삲�ƂɃX�e�b�v���s�����邩�ǂ���


	bool cmd_output = true;								//!< �R�}���h���C���ɏo�͂��邩�ǂ���
	OutputDetail cmd_permission = OutputDetail::kDebug;	//!< �R�}���h���C���ɏo�͂���ہC�ǂ��܂ŋ����邩
	bool gui_display = true;							//!< GUI��\�����邩�ǂ���
	std::string gui_display_quality = "high";			//!< GUI��\������ہC�ǂ��܂ŋ����邩
	int window_size_x = 1280;							//!< �O���t�B�J���E�B���h�E�̉���
	int window_size_y = 720;							//!< �O���t�B�J���E�B���h�E�̏c��
	int window_fps = 60; 								//!< �O���t�B�J���E�B���h�E��FPS	


	EMapCreateMode map_create_mode = EMapCreateMode::FLAT;	//!< �}�b�v�������[�h
	int map_create_option = 0;								//!< �}�b�v�����I�v�V����
	bool do_output_map = true;								//!< �}�b�v���o�͂��邩�ǂ���
	int map_hole_rate = 40;									//!< �}�b�v�̌��̊���
	float map_step_height = 500.0f;							//!< �}�b�v�̒i���̍���
	float map_step_length = -120.0f;						//!< �}�b�v�̒i���̒���
	float map_slope_angle_deg = 10.0f;						//!< �}�b�v�̍�̊p�x
	float map_tilt_angle_deg = 5.0f;						//!< �}�b�v�̌X���̊p�x
	float rough_max_dif = 30.0f;							//!< �}�b�v�̑e���̍ő�l
	float rough_min_dif = -30.0f;							//!< �}�b�v�̑e���̍ŏ��l

};

