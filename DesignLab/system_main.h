//! @file system_main.h
//! @brief ���̃v���O�����̏������܂Ƃ߂����́D�����̓��e��傫���ς������ꍇ��int Main()����C�S���ʂ̃N���X���Ăׂ΂悢�D

//! @class SystemMain
//! @brief ���`��K�͂Ȑ݌v�ɂ����āCint main�ɂȂ�ł��l�ߍ��ނ킯�ɂ͂����Ȃ����߁C���̃N���X�ɂ܂Ƃ߂�D
//! @details �����̓��e������������Ƃ��ɂ́Cint main()����ĂԃN���X��ς��邾���ł����D


#ifndef DESIGNLAB_SYSTEM_MAIN_H
#define DESIGNLAB_SYSTEM_MAIN_H

#include <memory>
#include <string>

#include "map_state.h"
#include "target.h"
#include "graphic_data_broker.h"
#include "graphic_system.h"
#include "abstract_pass_finder.h"
#include "interface_graphic_main_builder.h"
#include "abstract_graphic_main.h"
#include "abstract_hexapod_state_calculator.h"
#include "designlab_timer.h"
#include "result_file_exporter.h"
#include "application_setting_recorder.h"


class SystemMain final
{
public:
	SystemMain() = delete;		//�f�t�H���g�R���X�g���N�^�͋֎~�D

	//! @brief �R���X�g���N�^�D
	//! @param[in] graph_search ���e�v����s���N���X�D
	//! @param[in] graph_search_factory ���e�v����s���N���X�̃t�@�N�g���D
	//! @param[in] builder �O���t�B�b�N�̕`����s���N���X�D
	//! @param[in] calc �Z�r���s���{�b�g�̏�Ԃ��v�Z����N���X�D���ꂾ��shared_ptr�Ȃ̂Œ���
	SystemMain(std::unique_ptr<AbstractPassFinder>&& graph_search, std::unique_ptr<IPassFinderFactory>&& graph_search_factory,
		std::unique_ptr<IGraphicMainBuilder>&& builder, std::shared_ptr<AbstractHexapodStateCalculator> calc, SApplicationSettingRecorder* recorder);


	//! @brief ���܂܂�int main�ōs��ꂽ�������܂Ƃ߂����́D�ڕW�n�_�֒������C���e�v��Ɏ��s�����ꍇ�ɁC�V�~�����[�V�������I����D�K��̉񐔃V�~�����[�V����������I������D
	void Main();

private:

	void OutputTitle() const;

	void OutputSetting() const;


	MapState map_state_;

	STarget target_;

	GraphicDataBroker broker_;

	GraphicSystem graphic_system_;

	std::unique_ptr<AbstractPassFinder> pass_finder_ptr_;


	DesignlabTimer timer_;					//���Ԍv���p�̃N���X�D

	ResultFileExporter result_exporter_;	//���ʂ��t�@�C���ɏo�͂���N���X�D

	const SApplicationSettingRecorder* const mp_setting;	//�ݒ�t�@�C���̓��e���i�[����\���́D
};


#endif