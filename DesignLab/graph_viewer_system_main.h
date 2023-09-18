//! @file graph_viewer_system_main.h
//! @brief �O���t��\������V�X�e���̃��C���N���X

#ifndef DESIGNLAB_GRAPH_VIEWER_SYSTEM_MAIN_H_
#define DESIGNLAB_GRAPH_VIEWER_SYSTEM_MAIN_H_

#include <memory>

#include "map_state.h"
#include "graphic_system.h"
#include "graphic_data_broker.h"
#include "abstract_pass_finder.h"
#include "application_setting_recorder.h"


//! @class GraphViewerSystemMain
//! @brief �O���t��\������V�X�e���̃��C���N���X
//! @details ���̌����̎�@�ł͖؍\���̃O���t���쐬����D
//! �ǂ̂悤�ȃO���t���쐬����邩���m�F���邽�߂ɁC���̃O���t��\������V�X�e�����쐬�����D

class GraphViewerSystemMain final
{
public:

	GraphViewerSystemMain(
		std::unique_ptr<AbstractPassFinder>&& pass_finder_ptr,
		std::unique_ptr<IGraphicMain>&& graphic_main_ptr,
		const std::shared_ptr<GraphicDataBroker>& broker_ptr,
		const std::shared_ptr<const SApplicationSettingRecorder>& setting_ptr
	);

	//! @brief ���C���֐�
	void Main();

private:

	// �O���t���쐬����
	void CreateGraph(const SNode parent, std::vector<SNode>& graph);

	//�O���t�𒇉�l�ɃZ�b�g����
	void SetGraphToBroker(const std::vector<SNode>& graph);

	// y / n �̎��������
	bool askYesNo(const std::string& question) const;

	// �O���t�̃X�e�[�^�X��\������
	void showGraphStatus(const std::vector<SNode>& graph) const;


	GraphicSystem graphic_system_;

	std::unique_ptr<AbstractPassFinder> pass_finder_ptr_;

	const std::shared_ptr<GraphicDataBroker> broker_ptr_;

	const std::shared_ptr<const SApplicationSettingRecorder> setting_ptr_;

	MapState map_state_;
};


#endif // !DESIGNLAB_GRAPH_VIEWER_SYSTEM_MAIN_H_