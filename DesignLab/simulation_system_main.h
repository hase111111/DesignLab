//! @file simulation_system_main.h
//! @brief ���e�����V�~�����[�V�������s���N���X�D��s�����ɂ�����int main()�ōs���Ă����������܂Ƃ߂����́D

#ifndef SIMULATION_SYSTEM_MAIN_H_
#define SIMULATION_SYSTEM_MAIN_H_

#include <memory>
#include <string>

#include "abstract_hexapod_state_calculator.h"
#include "application_setting_recorder.h"
#include "graphic_data_broker.h"
#include "graphic_system.h"
#include "interface_graphic_main.h"
#include "interface_pass_finder.h"
#include "interface_system_main.h"
#include "map_state.h"
#include "result_file_exporter.h"
#include "stopwatch.h"
#include "target.h"


//! @class SimulationSystemMain
//! @brief ���`��K�͂Ȑ݌v�ɂ����āCint main�ɂȂ�ł��l�ߍ��ނ킯�ɂ͂����Ȃ����߁C���̃N���X�ɂ܂Ƃ߂�D
//! @details �����̓��e������������Ƃ��ɂ́Cint main()����ĂԃN���X��ς��邾���ł����D

class SimulationSystemMain final : public ISystemMain
{
public:
	SimulationSystemMain() = delete;		//�f�t�H���g�R���X�g���N�^�͋֎~�D

	//! @param[in] pass_finder_ptr ���R���e�p�^�[���������s���N���X�D
	//! @param[in] graphic_ptr �O���t�B�b�N��`�悷��N���X�D
	//! @param[in] broker_ptr �O���t�B�b�N�f�[�^���Ǘ�����N���X�D
	//! @param[in] setting_ptr �ݒ�t�@�C���̓��e���i�[����\���́D
	SimulationSystemMain(
		std::unique_ptr<IPassFinder>&& pass_finder_ptr,
		std::unique_ptr<IGraphicMain>&& graphic_ptr,
		const std::shared_ptr<GraphicDataBroker>& broker_ptr,
		const std::shared_ptr<const ApplicationSettingRecorder>& setting_ptr);


	//! @brief ���܂܂�int main�ōs��ꂽ�������܂Ƃ߂����́D
	//! @n �ڕW�n�_�֒������C���e�v��Ɏ��s�����ꍇ�ɁC�V�~�����[�V�������I����D
	//! @n �܂��C�K��̉񐔃V�~�����[�V����������I������D
	void Main() override;

private:

	void OutputSetting() const;


	std::unique_ptr<IPassFinder> pass_finder_ptr_;

	GraphicSystem graphic_system_;

	const std::shared_ptr<GraphicDataBroker> broker_ptr_;					//!< �O���t�B�b�N�f�[�^���Ǘ�����N���X�D

	const std::shared_ptr<const ApplicationSettingRecorder> setting_ptr_;	//!< �ݒ�t�@�C���̓��e���i�[����\���́D


	MapState map_state_;	//!< �n�`�̏�Ԃ��Ǘ�����N���X�D

	TargetRobotState target_;		//!< �ڕW�n�_�D

	Stopwatch timer_;		//!< ���Ԍv���p�̃N���X�D

	ResultFileExporter result_exporter_;	//!< ���ʂ��t�@�C���ɏo�͂���N���X�D
};


#endif