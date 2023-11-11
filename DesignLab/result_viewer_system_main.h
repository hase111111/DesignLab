//! @file result_viewer_system_main.h
//! @brief ���ʂ�\������V�X�e���̃N���X�D


#ifndef DESIGNLAB_RESULT_VIEWER_SYSTEM_MAIN_H_
#define DESIGNLAB_RESULT_VIEWER_SYSTEM_MAIN_H_


#include <memory>
#include <string>

#include "application_setting_recorder.h"
#include "graphic_data_broker.h"
#include "interface_system_main.h"
#include "result_file_importer.h"


//! @class ResultViewerSystemMain
//! @brief ���ʂ�\������V�X�e���̃N���X�D
class ResultViewerSystemMain final : public ISystemMain
{
public:
	ResultViewerSystemMain(
		const std::shared_ptr<GraphicDataBroker>& broker_ptr,
		const std::shared_ptr<const ApplicationSettingRecorder> setting_ptr
	);


	void Main() override;

private:

	ResultFileImporter result_importer_;

	const std::shared_ptr<GraphicDataBroker> broker_ptr_;
};


#endif	// DESIGNLAB_RESULT_VIEWER_SYSTEM_MAIN_H_