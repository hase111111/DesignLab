//! @file graph_viewer_system_main.h
//! @brief �O���t��\������V�X�e���̃��C���N���X

#ifndef DESIGNLAB_GRAPH_VIEWER_SYSTEM_MAIN_H_
#define DESIGNLAB_GRAPH_VIEWER_SYSTEM_MAIN_H_

#include <memory>

#include "application_setting_recorder.h"
#include "graphic_system.h"
#include "graphic_data_broker.h"
#include "interface_pass_finder.h"
#include "interface_system_main.h"
#include "map_state.h"


//! @class GraphViewerSystemMain
//! @brief �O���t��\������V�X�e���̃��C���N���X
//! @details ���̌����̎�@�ł͖؍\���̃O���t���쐬����D
//! �ǂ̂悤�ȃO���t���쐬����邩���m�F���邽�߂ɁC���̃O���t��\������V�X�e�����쐬�����D
class GraphViewerSystemMain final : public ISystemMain
{
public:

	GraphViewerSystemMain(
		std::unique_ptr<IPassFinder>&& pass_finder_ptr,
		std::unique_ptr<IGraphicMain>&& graphic_main_ptr,
		const std::shared_ptr<GraphicDataBroker>& broker_ptr,
		const std::shared_ptr<const ApplicationSettingRecorder>& setting_ptr
	);

	//! @brief ���C���֐�
	void Main() override;

private:

	// �O���t���쐬����
	void CreateGraph(const RobotStateNode parent, std::vector<RobotStateNode>& graph);

	//�O���t�𒇉�l�ɃZ�b�g����
	void SetGraphToBroker(const std::vector<RobotStateNode>& graph);

	// y / n �̎��������
	bool askYesNo(const std::string& question) const;

	// �O���t�̃X�e�[�^�X��\������
	void showGraphStatus(const std::vector<RobotStateNode>& graph) const;


	GraphicSystem graphic_system_;

	std::unique_ptr<IPassFinder> pass_finder_ptr_;

	const std::shared_ptr<GraphicDataBroker> broker_ptr_;

	const std::shared_ptr<const ApplicationSettingRecorder> setting_ptr_;

	MapState map_state_;
};


#endif // !DESIGNLAB_GRAPH_VIEWER_SYSTEM_MAIN_H_