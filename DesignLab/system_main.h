#pragma once

#include <memory>
#include <string>

#include "map_state.h"
#include "Target.h"
#include "graphic_data_broker.h"
#include "graphic_system.h"
#include "abstract_pass_finder.h"
#include "interface_graphic_main_builder.h"
#include "abstract_graphic_main.h"
#include "abstract_hexapod_state_calculator.h"
#include "designlab_timer.h"
#include "result_file_exporter.h"
#include "application_setting_recorder.h"



//! @class SystemMain
//! @date 2023/08/06
//! @author ���J��
//! @brief ���`��K�͂Ȑ݌v�ɂ����āCint main�ɂȂ�ł��l�ߍ��ނ킯�ɂ͂����Ȃ����߁C���̃N���X�ɂ܂Ƃ߂�D
//! @details �����̓��e������������Ƃ��ɂ́Cint main����ĂԃN���X��ς��邾���ł����D
class SystemMain final
{
public:
	SystemMain() = delete;		//�f�t�H���g�R���X�g���N�^�͋֎~�D

	SystemMain(std::unique_ptr<AbstractPassFinder>&& graph_search, std::unique_ptr<AbstractPassFinderFactory>&& graph_search_factory,
		std::unique_ptr<IGraphicMainBuilder>&& builder, std::shared_ptr<AbstractHexapodStateCalculator> calc, SApplicationSettingRecorder* recorder);

	~SystemMain() = default;


	//! @brief ���܂܂�int main�ōs��ꂽ�������܂Ƃ߂����́D�ڕW�n�_�֒������C���e�v��Ɏ��s�����ꍇ�ɁC�V�~�����[�V�������I����D�K��̉񐔃V�~�����[�V����������I������D
	void main();

private:

	void outputTitle() const;

	void outputSetting() const;


	MapState m_map_state;

	STarget m_target;

	GraphicDataBroker m_broker;

	GraphicSystem m_graphic_system;

	std::unique_ptr<AbstractPassFinder> mp_pass_finder;


	DesignlabTimer m_timer;					//���Ԍv���p�̃N���X�D

	ResultFileExporter m_result_exporter;	//���ʂ��t�@�C���ɏo�͂���N���X�D

	const SApplicationSettingRecorder* const mp_setting;	//�ݒ�t�@�C���̓��e���i�[����\���́D
};


//! @file system_main.h
//! @date 2023/08/06
//! @author ���J��
//! @brief ���̃v���O�����̏������܂Ƃ߂����́D�����̓��e��傫���ς������ꍇ��int main()����C�S���ʂ̃N���X���Ăׂ΂悢�D
//! @n �s�� : @lineinfo